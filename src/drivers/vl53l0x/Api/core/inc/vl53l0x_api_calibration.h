/*******************************************************************************
Copyright � 2016, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.
	* Neither the name of STMicroelectronics nor the
	  names of its contributors may be used to endorse or promote products
	  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef _VL53L0X_API_CALIBRATION_H_
#define _VL53L0X_API_CALIBRATION_H_

#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"


#ifdef __cplusplus
extern "C" {
#endif

VL53L0X_Error VL53L0X_perform_xtalk_calibration(VL53L0X_DEV Dev,
		FixPoint1616_t XTalkCalDistance,
		FixPoint1616_t *pXTalkCompensationRateMegaCps);

VL53L0X_Error VL53L0X_perform_offset_calibration(VL53L0X_DEV Dev,
		FixPoint1616_t CalDistanceMilliMeter,
		int32_t *pOffsetMicroMeter);

VL53L0X_Error VL53L0X_set_offset_calibration_data_micro_meter(VL53L0X_DEV Dev,
		int32_t OffsetCalibrationDataMicroMeter);

VL53L0X_Error VL53L0X_get_offset_calibration_data_micro_meter(VL53L0X_DEV Dev,
		int32_t *pOffsetCalibrationDataMicroMeter);

VL53L0X_Error VL53L0X_apply_offset_adjustment(VL53L0X_DEV Dev);

void get_next_good_spad(uint8_t goodSpadArray[], uint32_t size,
						uint32_t curr, int32_t *next);

uint8_t is_aperture(uint32_t spadIndex);

VL53L0X_Error enable_spad_bit(uint8_t spadArray[], uint32_t size,
							  uint32_t spadIndex);

VL53L0X_Error count_enabled_spads(uint8_t spadArray[],
								  uint32_t byteCount, uint32_t maxSpads,
								  uint32_t *pTotalSpadsEnabled, uint8_t *pIsAperture);

VL53L0X_Error set_ref_spad_map(VL53L0X_DEV Dev, uint8_t *refSpadArray);

VL53L0X_Error get_ref_spad_map(VL53L0X_DEV Dev, uint8_t *refSpadArray);

VL53L0X_Error enable_ref_spads(VL53L0X_DEV Dev,
							   uint8_t apertureSpads,
							   uint8_t goodSpadArray[],
							   uint8_t spadArray[],
							   uint32_t size,
							   uint32_t start,
							   uint32_t offset,
							   uint32_t spadCount,
							   uint32_t *lastSpad);

VL53L0X_Error perform_ref_signal_measurement(VL53L0X_DEV Dev,
											 uint16_t *refSignalRate);

VL53L0X_Error VL53L0X_perform_ref_spad_management(VL53L0X_DEV Dev,
		uint32_t *refSpadCount, uint8_t *isApertureSpads);

VL53L0X_Error VL53L0X_set_reference_spads(VL53L0X_DEV Dev,
		uint32_t count, uint8_t isApertureSpads);

VL53L0X_Error VL53L0X_get_reference_spads(VL53L0X_DEV Dev,
		uint32_t *pSpadCount, uint8_t *pIsApertureSpads);

VL53L0X_Error VL53L0X_perform_single_ref_calibration(VL53L0X_DEV Dev,
													 uint8_t vhv_init_byte);

VL53L0X_Error VL53L0X_ref_calibration_io(VL53L0X_DEV Dev, uint8_t read_not_write,
										 uint8_t VhvSettings, uint8_t PhaseCal,
										 uint8_t *pVhvSettings, uint8_t *pPhaseCal,
										 const uint8_t vhv_enable, const uint8_t phase_enable);

VL53L0X_Error VL53L0X_perform_vhv_calibration(VL53L0X_DEV Dev,
											  uint8_t *pVhvSettings, const uint8_t get_data_enable,
											  const uint8_t restore_config);

VL53L0X_Error VL53L0X_perform_phase_calibration(VL53L0X_DEV Dev,
	uint8_t *pPhaseCal, const uint8_t get_data_enable,
	const uint8_t restore_config);

VL53L0X_Error VL53L0X_perform_ref_calibration(VL53L0X_DEV Dev,
	uint8_t *pVhvSettings, uint8_t *pPhaseCal, uint8_t get_data_enable);

VL53L0X_Error VL53L0X_set_ref_calibration(VL53L0X_DEV Dev,
		uint8_t VhvSettings, uint8_t PhaseCal);

VL53L0X_Error VL53L0X_get_ref_calibration(VL53L0X_DEV Dev,
		uint8_t *pVhvSettings, uint8_t *pPhaseCal);




#ifdef __cplusplus
}
#endif

#endif /* _VL53L0X_API_CALIBRATION_H_ */
