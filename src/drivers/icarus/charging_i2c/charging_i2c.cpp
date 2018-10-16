

/**
 * @file charging_i2c.cpp
 * @author Gabriel Guilmain
 *
 * Driver for MAX17205 (need custom QgroundControl Builds to monitor battery parameters)
 */

#include <px4_config.h>
#include <px4_defines.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/icarus/charging_i2c/charging_i2c.h>

#include <uORB/uORB.h>

// on inclut le nouveau UORB topic spécifique pour le driver charging i2c
#include <uORB/topics/battery_status.h>
#include <uORB/topics/charging_info.h>

#include <board_config.h>

/* Configuration Constants */
#define MAX17205_I2C_BUS 		PX4_I2C_BUS_ONBOARD
#define MAX17205_I2C_BASEADDR 	0x36 /* vaut 0x6C selon la convention du chip de balancing. ladresse est tassé vers la droite et le dernier bit est sous-entendu */
#define MAX17205_DEVICE_PATH	"/dev/charging_i2c"
#define MAX17205_CONVERSION_INTERVAL 	100000 /* 60ms for one sonar */
#define TICKS_BETWEEN_SUCCESIVE_FIRES 	100000 /* 30ms between each sonar measurement (watch out for interference!) */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class CHARGING_I2C : public device::I2C
{
public:
	CHARGING_I2C(int bus = MAX17205_I2C_BUS, int address = MAX17205_I2C_BASEADDR);
	virtual ~CHARGING_I2C();
	virtual int 		init();
	virtual ssize_t     read(struct file *filp, charging_info_s *report, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);
	void				print_info();
	void				checkEeprom();
	void 				gaugereset();

protected:
	virtual int			probe();

private:

	struct {
		float is_new_bat;
		float bat_cap;
		float rsense;
		float balance_threshold;
	}	_parameters;

	struct {
		param_t is_new_bat;
		param_t bat_cap;
		param_t rsense;
		param_t balance_threshold;
	}	_parameter_handles;		/**< handles for interesting parameters */

	work_s				_work;
	ringbuffer::RingBuffer		*_reports;
	bool				_sensor_ok;
	int					_measure_ticks;
	bool				_collect_phase;
	int					_class_instance;
	int					_orb_class_instance;

	// topic custom pour charging i2c
	orb_advert_t     	_charging_info_topic;
	orb_advert_t		_battery_pub;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uint8_t				_cycle_counter;	/* counter in cycle to change i2c adresses */
	int				_cycling_rate;	/* */
	uint8_t				_index_counter;	/* temporary sonar i2c address */
	std::vector<uint8_t>	addr_ind; 	/* temp sonar i2c address vector */

	struct charging_info_s last_report;
	float bat_full_cap;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();


	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	int					collect();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" { __EXPORT int charging_i2c_main(int argc, char *argv[]);}

CHARGING_I2C::CHARGING_I2C(int bus, int address) :
		I2C("MAX17205", MAX17205_DEVICE_PATH, bus, address, 100000), // _address est private on appelle donc le constructor de la classe mere
		// la frequence du bus est à 100 khz ***
		_reports(nullptr),
		_sensor_ok(false),
		_measure_ticks(0),
		_collect_phase(false),
		_class_instance(-1),
		_orb_class_instance(-1),
		// _battery_pub(nullptr),
		_charging_info_topic(nullptr),	// charging_i2c custom topic
		_sample_perf(perf_alloc(PC_ELAPSED, "charging_i2c_read")),
		_comms_errors(perf_alloc(PC_COUNT, "charging_i2c_comms_errors")),
		_cycle_counter(0),	/* initialising counter for cycling function to zero */
		_cycling_rate(0),	/* initialising cycling rate (which can differ depending on one sonar or multiple) */
		_index_counter(0) 	/* initialising temp sonar i2c address to zero */

{
	_parameter_handles.is_new_bat = param_find("IS_NEW_BATTERY");
	_parameter_handles.bat_cap = param_find("BATT_CAPA_MAH");
	_parameter_handles.rsense = param_find("RSENSE_MOHM");
	_parameter_handles.balance_threshold = param_find("BALANCE_THRESH");

	parameters_update();

	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

// constructor utilse initialisation list. cest plus performant...

CHARGING_I2C::~CHARGING_I2C()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(MAX17205_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

//********************************************//
// PARAMETER UPDATE                         ////
//********************************************//
int
CHARGING_I2C::parameters_update()
{

	param_get(_parameter_handles.is_new_bat, &_parameters.is_new_bat);
	param_get(_parameter_handles.bat_cap, &_parameters.bat_cap);
	param_get(_parameter_handles.rsense, &_parameters.rsense);
	param_get(_parameter_handles.balance_threshold, &_parameters.balance_threshold);

	return OK;
}

int
CHARGING_I2C::init()
{
	int ret = PX4_ERROR; // -1 -> valeur standard

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		warn("I2C init failed (debug)");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(charging_info_s));

	/* Set I2C Address */
	_index_counter = MAX17205_I2C_BASEADDR;	/* set temp sonar i2c address to base adress */
	set_device_address(_index_counter);		/* set I2c port to temp sonar i2c adress */

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(MAX17205_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct charging_info_s ds_report = {};
	struct battery_status_s battery_report = {};

	_battery_pub = orb_advertise_multi(ORB_ID(battery_status), &battery_report,
											 &_orb_class_instance, ORB_PRIO_VERY_HIGH);

	_charging_info_topic = orb_advertise_multi(ORB_ID(charging_info), &ds_report,
											   &_orb_class_instance, ORB_PRIO_LOW);

	


	if (_charging_info_topic == nullptr) {
		DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
	}

	// XXX we should find out why we need to wait 200 ms here
	usleep(200000);

	// on enleve MB12XX_MAX_RANGEFINDERS comme condition de la boucle suivante
	// permettait de faire un scroll de ladresse avec plusieurs essaie mais on en a pas besoin
	for (unsigned counter = 0; counter <= 0; counter++) {
		_index_counter = MAX17205_I2C_BASEADDR + counter * 2;	/* set temp sonar i2c address to base adress - counter */
		set_device_address(_index_counter);				/* set I2c port to temp sonar i2c adress */

		//TODO: Ajouter la detection du module MAX17205

		// pour debug notre device repond a quel adresse (en base 10)
		warn("ladresse est (BASE 10): %i", _index_counter);
	}

	_index_counter = MAX17205_I2C_BASEADDR;
	set_device_address(_index_counter); /* set i2c port back to base adress for rest of driver */

	// cycling rate utilisé (dans notre cas, il y a un seul capteur detecté)
	if (addr_ind.size() == 1) {
		_cycling_rate = MAX17205_CONVERSION_INTERVAL;

	} else {
		_cycling_rate = TICKS_BETWEEN_SUCCESIVE_FIRES;
	}

	/* show the connected device in terminal */
	for (unsigned i = 0; i < addr_ind.size(); i++) {
		DEVICE_LOG("sonar %d with address %d added", (i + 1), addr_ind[i]);
	}

	ret = OK;

	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	return ret;
}

int
CHARGING_I2C::probe()
{
	warnx("probe class charging i2c (debug)"); // debug erreur
	// measure() dans la classe charging I2C
	//return measure();
	return OK;
}

int
CHARGING_I2C::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

		case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
				case SENSOR_POLLRATE_MANUAL:
					stop();
					_measure_ticks = 0;
					return OK;

					/* external signalling (DRDY) not supported */
				case SENSOR_POLLRATE_EXTERNAL:

					/* zero would be bad */
				case 0:
					return -EINVAL;

					/* set default/max polling rate */
				case SENSOR_POLLRATE_MAX:

					////////////////////////////////////////////////////////////////
					// CEST LE CASE OU ON VA DANS LES CONDITIONS DE BASE
					// SANS RIEN MODIFIER DU CODE -> ON LAIISERA CA AINSI
					////////////////////////////////////////////////////////////////

				case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_cycling_rate);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

					/* adjust to a legal polling interval in Hz */
				default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_cycling_rate)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

		case SENSORIOCGPOLLRATE:
			if (_measure_ticks == 0) {
				return SENSOR_POLLRATE_MANUAL;
			}

			return (1000 / _measure_ticks);

		case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

		case SENSORIOCRESET:
			/* XXX implement this */
			return -EINVAL;


		default:
			/* give it to the superclass */
			return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
CHARGING_I2C::read(struct file *filp, charging_info_s *report, size_t buflen)
{

	// on met lasresse pour faire un read des parametres (0x6C selon la chip -> 0x36 selon la nomenclature du driver)
	set_device_address(0x36); /* Address in 7bit format (0x6C -> 0x36)*/

	// commandes pour lire les valeurs pertinentes sur le MAX17205 (voir plus bas pour les valeurs lues)
	uint8_t cmd[11] = {0x19,0xD2,0xD3,0xD4,0x05,0x0A,0x0B,0x11,0x20,0x06,0xB8};

	// valeurs à lire sur le MAX17205
	uint8_t val1[2] = {0,0};
	uint8_t val2[2] = {0,0};
	uint8_t val3[2] = {0,0};
	uint8_t val4[2] = {0,0};
	uint8_t val5[2] = {0,0};
	uint8_t val6[2] = {0,0};
	uint8_t val7[2] = {0,0};
	uint8_t val8[2] = {0,0};
	uint8_t val9[2] = {0,0};
	uint8_t val10[2] = {0,0};

	perf_begin(_sample_perf);

	// lectures des registres
	transfer(&cmd[0], 1, val1, 2); // envoie la commande au slave pour faire un read AvgVCell
	transfer(&cmd[1], 1, val2, 2); // envoie la commande au slave pour faire un read AvgCell3
	transfer(&cmd[2], 1, val3, 2); // envoie la commande au slave pour faire un read AvgCell2
	transfer(&cmd[3], 1, val4, 2); // envoie la commande au slave pour faire un read AvgCell1
	transfer(&cmd[4], 1, val5, 2); // envoie la commande au slave pour faire un read RepCap
	transfer(&cmd[5], 1, val6, 2); // envoie la commande au slave pour faire un read Current
	transfer(&cmd[6], 1, val7, 2); // envoie la commande au slave pour faire un read AvgCurrent
	transfer(&cmd[7], 1, val8, 2); // envoie la commande au slave pour faire un read TTE (Time To Empty)
	transfer(&cmd[8], 1, val9, 2); // envoie la commande au slave pour faire un read TTF (Time To Full)
	transfer(&cmd[9], 1, val10, 2); // envoie la commande au slave pour faire un read	RepSOC



	// publish des valeurs lu sur le MAX17205 (et conditionnement des valeurs) (VOIR TABLEAU 1 PAGE 26 DE LA DATASHEET)
	report->avg_vcell = (float)(((int)val1[1] * 256) | (int)val1[0])/12800.0f;	// average cell voltage (V)
	report->avg_vcell3 = (float)(((int)val2[1] * 256) | (int)val2[0])/12800.0f;	// cell 3 voltage (V)
	report->avg_vcell2 = (float)(((int)val3[1] * 256) | (int)val3[0])/12800.0f;	// cell 2 voltage (V)
	report->avg_vcell1 = (float)(((int)val4[1] * 256) | (int)val4[0])/12800.0f;	// cell 1 voltage (V)

	report->rep_cap = (float)(((int)val5[1] * 256) | (int)val5[0])*((5.0f/1000.0f)/(_parameters.rsense / 1000.0f));		// reported capacity (mAh)

	report->current = (float)(int16_t((val6[1] * 256) | val6[0]))*((1.5625f/1000.0f)/(_parameters.rsense / 1000.0f));	// current (mA)
	report->avg_current = (float)(int16_t((val7[1] * 256) | val7[0]))*((1.5625f/1000.0f)/(_parameters.rsense / 1000.0f));		// average current (mA)

	report->tte = (float)(((int)val8[1] * 256) | (int)val8[0])/640.0f;		// time to empty (hr)
	report->ttf = (float)(((int)val9[1] * 256) | (int)val9[0])/640.0f;		// time to full (hr)
	report->rep_soc = (float)(((int)val10[1] * 256) | (int)val10[0])/256.0f;		// reported state of charge (%)

	return buflen;
}


int
CHARGING_I2C::collect()
{
	//la fonction transfer fonctionne ainsi (VOIR I2C_NUTTX.H)
	/**
	 * Perform an I2C transaction to the device.
	 *
	 * At least one of send_len and recv_len must be non-zero.
	 *
	 * @param send		Pointer to bytes to send.
	 * @param send_len	Number of bytes to send.
	 * @param recv		Pointer to buffer for bytes received.
	 * @param recv_len	Number of bytes to receive.
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 **/

	int ret = -EIO;

	struct charging_info_s report;
	struct battery_status_s battery_report;

	// update parameters for config or reading of MAX17205
	parameters_update();

	/////////////////////////////////////////////////////////////////////////////////////////
	// LECTURE EN BOUCLE DES REGISTRES PERTINENT DU MAX17205 (CHIP DE BALANCING DES CELLULES)
	/////////////////////////////////////////////////////////////////////////////////////////
	if((int)(_parameters.is_new_bat) == 0)
	{
		// on met lasresse pour faire un read des parametres (0x6C selon la chip -> 0x36 selon la nomenclature du driver)
		set_device_address(0x36); /* Address in 7bit format (0x6C -> 0x36)*/

		// commandes pour lire les valeurs pertinentes sur le MAX17205 (voir plus bas pour les valeurs lues)
		uint8_t cmd[11] = {0x19,0xD2,0xD3,0xD4,0x05,0x0A,0x0B,0x11,0x20,0x06,0X35};

		// valeurs à lire sur le MAX17205
		uint8_t val1[2] = {0,0};
		uint8_t val2[2] = {0,0};
		uint8_t val3[2] = {0,0};
		uint8_t val4[2] = {0,0};
		uint8_t val5[2] = {0,0};
		uint8_t val6[2] = {0,0};
		uint8_t val7[2] = {0,0};
		uint8_t val8[2] = {0,0};
		uint8_t val9[2] = {0,0};
		uint8_t val10[2] = {0,0};
		uint8_t val11[2] = {0,0};

		perf_begin(_sample_perf);

		// lectures des registres
		ret = transfer(&cmd[0], 1, val1, 2); // envoie la commande au slave pour faire un read AvgVCell
		ret = transfer(&cmd[1], 1, val2, 2); // envoie la commande au slave pour faire un read AvgCell3
		ret = transfer(&cmd[2], 1, val3, 2); // envoie la commande au slave pour faire un read AvgCell2
		ret = transfer(&cmd[3], 1, val4, 2); // envoie la commande au slave pour faire un read AvgCell1
		ret = transfer(&cmd[4], 1, val5, 2); // envoie la commande au slave pour faire un read RepCap
		ret = transfer(&cmd[5], 1, val6, 2); // envoie la commande au slave pour faire un read Current
		ret = transfer(&cmd[6], 1, val7, 2); // envoie la commande au slave pour faire un read AvgCurrent
		ret = transfer(&cmd[7], 1, val8, 2); // envoie la commande au slave pour faire un read TTE (Time To Empty)
		ret = transfer(&cmd[8], 1, val9, 2); // envoie la commande au slave pour faire un read TTF (Time To Full)
		ret = transfer(&cmd[9], 1, val10, 2); // envoie la commande au slave pour faire un read	RepSOC
		ret = transfer(&cmd[10], 1, val11, 2); // envoie la commande au slave pour faire un read	RepSOC

		//ret = OK;

		if (ret < 0) {
			DEVICE_DEBUG("error reading from sensor: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}

		// publish des valeurs lu sur le MAX17205 (et conditionnement des valeurs) (VOIR TABLEAU 1 PAGE 26 DE LA DATASHEET)
		report.avg_vcell = (float)(((int)val1[1] * 256) | (int)val1[0])/12800.0f;	// average cell voltage (V)
		report.avg_vcell3 = (float)(((int)val2[1] * 256) | (int)val2[0])/12800.0f;	// cell 3 voltage (V)
		report.avg_vcell2 = (float)(((int)val3[1] * 256) | (int)val3[0])/12800.0f;	// cell 2 voltage (V)
		report.avg_vcell1 = (float)(((int)val4[1] * 256) | (int)val4[0])/12800.0f;	// cell 1 voltage (V)

		report.rep_cap = (float)(((int)val5[1] * 256) | (int)val5[0])*((5.0f/1000.0f)/(_parameters.rsense / 1000.0f));		// reported capacity (mAh)
		bat_full_cap = (float)(((int)val11[1] * 256) | (int)val11[0])*((5.0f/1000.0f)/(_parameters.rsense / 1000.0f));
		// bat_full_cap = _parameters.bat_cap;
		report.current = (float)(int16_t((val6[1] * 256) | val6[0]))*((1.5625f/1000.0f)/(_parameters.rsense / 1000.0f));	// current (mA)
		report.avg_current = (float)(int16_t((val7[1] * 256) | val7[0]))*((1.5625f/1000.0f)/(_parameters.rsense / 1000.0f));		// average current (mA)

		report.tte = (float)(((int)val8[1] * 256) | (int)val8[0])/640.0f;		// time to empty (hr)
		report.ttf = (float)(((int)val9[1] * 256) | (int)val9[0])/640.0f;		// time to full (hr)
		report.rep_soc = (float)(((int)val10[1] * 256) | (int)val10[0])/256.0f;
		// report.rep_soc = report.rep_cap/(_parameters.bat_cap*1.0f) * 100;	// reported state of charge (%)

		battery_report.voltage_v = report.avg_vcell3 + report.avg_vcell2 + report.avg_vcell1;
		battery_report.voltage_filtered_v = battery_report.voltage_v;
		battery_report.current_a = report.current / 10.0f;
		battery_report.current_filtered_a = battery_report.current_a;
		battery_report.average_current_a = report.avg_current / 10.0f;
		battery_report.discharged_mah = bat_full_cap - report.rep_cap;
		battery_report.remaining = report.rep_soc / 100.0f;
		battery_report.scale = -1;
		battery_report.temperature = 0;
		battery_report.cell_count = 3;
		battery_report.connected = true;
		battery_report.system_source = true;
		battery_report.priority = 0;
		battery_report.capacity = report.rep_cap;
		battery_report.cycle_count = -1;
		battery_report.run_time_to_empty = report.tte*3600;
		battery_report.average_time_to_empty = 0;
		battery_report.serial_number = 0;
		battery_report.is_powering_off = false;
		if(report.avg_vcell1 < 3.5f ||
			report.avg_vcell1 < 3.5f ||
			report.avg_vcell1 < 3.5f)
		{
			battery_report.warning = battery_report.BATTERY_WARNING_LOW;
		} 
		else if(report.avg_vcell1 < 3.3f ||
			report.avg_vcell1 < 3.3f ||
			report.avg_vcell1 < 3.3f)
		{
			battery_report.warning = battery_report.BATTERY_WARNING_CRITICAL;
		} 
		else if(report.avg_vcell1 < 3.1f ||
			report.avg_vcell1 < 3.1f ||
			report.avg_vcell1 < 3.1f)
		{
			battery_report.warning = battery_report.BATTERY_WARNING_EMERGENCY;
		} 
		else
		{
			battery_report.warning = battery_report.BATTERY_WARNING_NONE;
		}

		last_report = report;

	}
		/////////////////////////////////////////////////////////////////////////////////////////
		// CONFIGURATION DU MAX17205 SI LE PARAMETRE is_new_bat EST A 1
		/////////////////////////////////////////////////////////////////////////////////////////
    else if((int)(_parameters.is_new_bat) == 1)
    {
        /* Safety */
        usleep(10000000);

        uint8_t data1,data2,addressreg;
        uint8_t dataArray[3];

        set_device_address(0x0B);

        for(int i = 0; i<95; i++)
        {
            data1 = register_data[i] & 0x00FF;
            data2 = (register_data[i] & 0xFF00)>>8;
            addressreg = register_address[i] & 0x00FF;

            dataArray[0] = addressreg;
            dataArray[1] = data1;
            dataArray[2] = data2;

            transfer(dataArray, 3, nullptr, 0);
        }

        set_device_address(0x36);

        /* Clear the ClearComm.NVerror register */
        uint8_t DataToSend_1[3] = {0x61,0x00,0x00};
        transfer(DataToSend_1, 3, nullptr, 0);

        /* Send 0xE904 to the command register to initiate a block command write */
        uint8_t DataToSend_2[3] = {0x60,0x04,0xE9};
        transfer(DataToSend_2, 3, nullptr, 0);

        /* Wait for the copy to complete */
        usleep(5000000);

        /*Check if EEPROM write was successful */
        uint8_t buffer[2] = {0,0};
        uint8_t regcommstat[1] = {0x60};
        ret = transfer(regcommstat, 1, buffer, 2); // Fait une lecture de l'etat du NV register
        uint16_t NVStatus = ((uint16_t)((buffer[1] * 256) | buffer[0]) & 0b0000000000000100)>>2;
        if(NVStatus )
        {
            warnx("NV ERROR ACTIVE");
        }

        /* Send 0x000F to the command register to POR the IC*/
        uint8_t DataToSend_3[3] = {0x60,0x0F,0x00};
        transfer(DataToSend_3, 3, nullptr, 0);

        /* Wait for firmware reboot */
        usleep(5000000);

        /* Execute a POR Reboot */
        uint8_t DataToSend_4[3] = {0xBB,0x01,0x00};
        transfer(DataToSend_4, 3, nullptr, 0);

        /* TPOR is 10ms */
        usleep(5000000);

        /* Reset is_new_bat value to 0 */
        _parameters.is_new_bat = 0;
        param_set(_parameter_handles.is_new_bat, &_parameters.is_new_bat);

	}
		///////////////////////////[b]///////////////////////
		// PARAMETRE is_new_batt NON VALIDE (on)
		/////////////////////////////////////////////////////////////////////////////////////////
	else
	{
		perf_begin(_sample_perf);

		// publish des valeurs lu sur le MAX17205 (et conditionnement des valeurs) (VOIR TABLEAU 1 PAGE 26 DE LA DATASHEET)
		//struct charging_info_s report;
		report.avg_vcell = 0;
		report.avg_vcell3 = 0;
		report.avg_vcell2 = 0;
		report.avg_vcell1 = 0;
		report.rep_cap = 0;
		report.current = 0;
		report.avg_current = 0;
		report.tte = 0;
		report.ttf = 0;
		report.rep_soc = 0;
	}

	if (_battery_pub != nullptr){
		orb_publish(ORB_ID(battery_status), _battery_pub, &battery_report);
	}

	/* publish it, if we are the primary */
	if (_charging_info_topic != nullptr) {
		orb_publish(ORB_ID(charging_info), _charging_info_topic, &report);
	}


	_reports->force(&battery_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
CHARGING_I2C::start()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	// warn("fonction start work_queue");

	/* schedule a cycle to start things */
	//
	work_queue(HPWORK, &_work, (worker_t)&CHARGING_I2C::cycle_trampoline, this, 5);

}

void
CHARGING_I2C::stop()
{
	work_cancel(HPWORK, &_work);
}

void
CHARGING_I2C::cycle_trampoline(void *arg)
{

	// warn("fonction cycle trampoline");
	CHARGING_I2C *dev = (CHARGING_I2C *)arg;

	dev->cycle();

}

void
CHARGING_I2C::cycle()
{

	if (_collect_phase) {
		_index_counter = addr_ind[_cycle_counter]; /*sonar from previous iteration collect is now read out */
		set_device_address(_index_counter);

		/* perform collection */
		if (OK != collect()) {
			DEVICE_DEBUG("collection error");
			/* if error restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/* change i2c adress to next sonar */
		_cycle_counter = _cycle_counter + 1;

		if (_cycle_counter >= addr_ind.size()) {
			_cycle_counter = 0;
		}

		/* Is there a collect->measure gap? Yes, and the timing is set equal to the cycling_rate
		   Otherwise the next sonar would fire without the first one having received its reflected sonar pulse */

		if (_measure_ticks > USEC2TICK(_cycling_rate)) {

			// test debug
			//warn("condition 1? (debug)");
			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
					   &_work,
					   (worker_t)&CHARGING_I2C::cycle_trampoline,
					   this,
					   _measure_ticks - USEC2TICK(_cycling_rate));
			return;
		}
	}

	/* Measurement (firing) phase */

	/* ensure sonar i2c adress is still correct */
	_index_counter = addr_ind[_cycle_counter];
	set_device_address(_index_counter);

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	// test debug
	//warn("condition 2? (debug)");
	work_queue(HPWORK,
			   &_work,
			   (worker_t)&CHARGING_I2C::cycle_trampoline,
			   this,
			   USEC2TICK(_cycling_rate));

}

void
CHARGING_I2C::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("average cell voltage : %.2f V\n", (double)last_report.avg_vcell);
	printf("cell 1 voltage : %.2f V\n", (double)(last_report.avg_vcell1));
	printf("cell 2 voltage : %.2f V\n", (double)(last_report.avg_vcell2));
	printf("cell 3 voltage : %.2f V\n", (double)(last_report.avg_vcell3));
	printf("capacity : %f mAh\n",(double)(last_report.rep_cap));
	printf("full capacity : %f mAh\n",(double)(bat_full_cap));
	printf("current : %.2f mA\n",(double)(last_report.current));
	printf("average current : %.2f mA\n",(double)(last_report.avg_current));
	printf("time to empty : %f hr\n",(double)(last_report.tte));
	printf("time to full : %f hr\n",(double)(last_report.ttf));
	printf("state of charge : %f %%\n",(double)(last_report.rep_soc));

	_reports->print_info("report queue");


}

void
CHARGING_I2C::gaugereset()
{
	/* Set I2C adress */
    set_device_address(0x36);

    /* Send a command to reset the Fuel Gauge */
    uint8_t DataToSend[3] = {0xBB,0x01,0x00};
    transfer(DataToSend, 3, nullptr, 0);
}

void
CHARGING_I2C::checkEeprom()
{
    /* Set I2C Address*/
    set_device_address(0x36);

    /* Sends NV number of tries remaining command */
    uint8_t DataToSend_10[3] = {0x60,0xFA,0xE2};
    transfer(DataToSend_10, 3, nullptr, 0);

    /* Set I2C Address */
    set_device_address(0x0B);

    /* Check for Power status */
    uint8_t dataret[2] = {0,0};
    uint8_t reg[1] = {0xED};
    transfer(reg, 1, dataret, 2);

    int WriteLeft = 255;

    /* Calculate Number of remaining writes */
    if((int)dataret[0] == 0b00000000)
    {
        WriteLeft = 8;
    }
    else if((int)dataret[0] == 0b00000001)
    {
        WriteLeft = 7;
    }
    else if((int)dataret[0] == 0b00000011)
    {
        WriteLeft = 6;
    }
    else if((int)dataret[0] == 0b00000111)
    {
        WriteLeft = 5;
    }
    else if((int)dataret[0] == 0b00001111)
    {
        WriteLeft = 4;
    }
    else if((int)dataret[0] == 0b00011111)
    {
        WriteLeft = 3;
    }
    else if((int)dataret[0] == 0b00111111)
    {
        WriteLeft = 2;
    }
    else if((int)dataret[0] == 0b01111111)
    {
        WriteLeft = 1;
    }
    else if((int)dataret[0] == 0b11111111)
    {
        WriteLeft = 0;
    }

    if(WriteLeft != 255)
    {
        warnx("Number of EEPROM Writes left : %d",WriteLeft);
    }
    else
    {
        warnx("EEPROM read error !");
    }

    return;
}

/**
 * Local functions in support of the shell command
 * Pour appeler les fonctions on écrit:
 * charging_i2c start
 * charging_i2c stop
 * charging_i2c test
 * charging_i2c config
 * charging_i2c reset
 * charging_i2c info
  */
namespace  charging_i2c
{

	CHARGING_I2C	*g_dev;

	void	start();
	void	stop();
	void	test();
	void	reset();
	void	info();
	void	gaugereset();

/**
 * Start the driver.
 */
	void
	start()
	{
		int fd;

		if (g_dev != nullptr) {
			errx(1, "already started");
		}

		/////////////////////////////////////////////////////////////////////////////////////
		/* create the driver */
		// constructor de la classe fille appelle le contructor de la fonction mere
		/////////////////////////////////////////////////////////////////////////////////////
		g_dev = new CHARGING_I2C(MAX17205_I2C_BUS);

		if (g_dev == nullptr) {

			warnx("1"); // debug erreur
			goto fail;
		}

		/////////////////////////////////////////////////////////////////////////////////////
		// fonction init dans la classe fille (charging_i2c)
		// init fait appel aussi a init() de la classe mere. on set plusieurs chose:
		// - vitesse de bus
		// - probe() pour tester la presence du slave
		// - pour le capteur sonar a partir duquelon developpe le driver, fait un scan dadresse pour detecter plusieur capteurs
		// - initialise le UORB (a changer pour utiliser notre topic custom)
		// - prend la frequence de poll spécifique pour le capteur (sur loscilloscope semble etre 10 Hz)
		// TODO: modifier cette fonction pour effectuer la configuration des registres dans le chip de balancing
		/////////////////////////////////////////////////////////////////////////////////////
		if (OK != g_dev->init()) {

			warnx("2"); // debug erreur
			goto fail;
		}

		/////////////////////////////////////////////////////////////////////////////////////
		/* set the poll rate to default, starts automatic data collection */
		// ne comprend pas trop: si on va dans nsh SRF02_DEVICE_PATH (quon a setter a dev/charging_i2c )
		// apparait lorsque lon fait charging_i2c start. on uvre donc le peripherique qui fait mainteant partie de la liste
		// de tout les peripherique connecté au pixhawk, mais est-ce vraiment utile ?? -> a voir
		/////////////////////////////////////////////////////////////////////////////////////
		fd = open(MAX17205_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {

			warnx("3"); // debug erreur
			goto fail;
		}

		/////////////////////////////////////////////////////////////////////////////////////
		// fonction qui permet deffectuer lechantillonnage du capteur a la frequence desirée
		// TODO: determiner comment cette fonction permet dappeler les fonction de mesure en boucle
		// et changer ces fonction pour notre application
		// NOTE: selon les tests avec arduino et oscilloscope on semble appeler ::collect et apres :: measure
		// appelle la fonction charging_i2c::start selon certain case lié au parametres
		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {

			warnx("4"); // debug erreur
			goto fail;
		}

		warn("function start executed completely");
		exit(0);

		fail:

		if (g_dev != nullptr) {
			delete g_dev;
			g_dev = nullptr;
		}

		errx(1, "driver start failed");
	}

/**
 * Stop the driver
 */
	void stop()
	{
		if (g_dev != nullptr) {
			delete g_dev;
			g_dev = nullptr;

		} else {
			errx(1, "driver not running");
		}

		exit(0);
	}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
	void
	test()
	{
		struct charging_info_s report;
		ssize_t sz;

		int fd = open(MAX17205_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			err(1, "%s open failed (try 'charging_i2c start' if the driver is not running", MAX17205_DEVICE_PATH);
		}

		/* do a simple demand read */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "immediate read failed 1 %d %d",sz,sizeof(report));
		}

		warnx("single read");
		warnx("measurement total: %0.2f m", (double)report.avg_vcell);
		warnx("measurement cell 1: %0.2f m", (double)report.avg_vcell1);
		warnx("measurement cell 2: %0.2f m", (double)report.avg_vcell2);
		warnx("measurement cell 3: %0.2f m", (double)report.avg_vcell3);
		warnx("time:        %llu", report.timestamp);
	}

/**
 * Reset the driver.
 */
	void
	reset()
	{
		int fd = open(MAX17205_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			err(1, "failed ");
		}

		if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
			err(1, "driver reset failed");
		}

		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			err(1, "driver poll restart failed");
		}

		exit(0);
	}

/**
 * Print a little info about the driver.
 */
	void
	info()
	{
		if (g_dev == nullptr) {
			errx(1, "driver not running");
		}

		printf("state @ %p\n", g_dev);
		g_dev->print_info();

		g_dev->checkEeprom();

		exit(0);
	}

/**
 * Sends a command to the MAX17205 for a fuel gauge reset 
 */
	void
	gaugereset()
	{
		if (g_dev == nullptr) {
			errx(1, "driver not running");
		}

		printf("Resetting Fuel Gauge");
		g_dev->gaugereset();

		exit(0);	
	}

} /* namespace */

int
charging_i2c_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		charging_i2c::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		charging_i2c::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		charging_i2c::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		charging_i2c::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		charging_i2c::info();
	}

	/*
	 * Send a Gauge Reset command
	 */
	if(!strcmp(argv[1], "gaugereset")){
		charging_i2c::gaugereset();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' , 'gaugereset' or 'info'");
}