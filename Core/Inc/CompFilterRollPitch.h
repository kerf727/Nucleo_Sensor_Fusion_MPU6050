/*
 * CompFilterRollPitch.h
 *
 *  Created on: Jan 20, 2023
 *      Author: Kyle Erf
 */

#ifndef INC_COMPFILTERROLLPITCH_H_
#define INC_COMPFILTERROLLPITCH_H_

#define G_MPS2 9.80665f

#include "stdint.h"
#include "math.h"
#include "FirstOrderIIR.h"

typedef struct
{
	float comp_alpha;
	float sample_time_ms;
	float roll_rad;
	float pitch_rad;

	FirstOrderIIR lpf_acc[3];
	FirstOrderIIR lpf_gyr[3];

} CompFilterRollPitch;

void CompFilterRollPitch_Init(CompFilterRollPitch *filt, float alpha, float sample_time_ms, float lpf_acc_alpha, float lpf_gyr_alpha);
void CompFilterRollPitch_Update(CompFilterRollPitch *filt, float *acc_mps2, float *gyr_rps);

#endif /* INC_COMPFILTERROLLPITCH_H_ */
