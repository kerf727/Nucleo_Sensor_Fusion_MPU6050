#include "CompFilterRollPitch.h"

void CompFilterRollPitch_Init(CompFilterRollPitch *filt, float comp_alpha, float sample_time_ms, float lpf_acc_alpha, float lpf_gyr_alpha)
{
	// Set struct parameters
	filt->comp_alpha = comp_alpha;
	filt->sample_time_ms = sample_time_ms;
	filt->roll_rad = 0.0f;
	filt->pitch_rad = 0.0f;

	// Initialize First Order IIR filters for XL and gyro
	FirstOrderIIR_Init(&filt->lpf_acc[0], lpf_acc_alpha);
	FirstOrderIIR_Init(&filt->lpf_acc[1], lpf_acc_alpha);
	FirstOrderIIR_Init(&filt->lpf_acc[2], lpf_acc_alpha);
	FirstOrderIIR_Init(&filt->lpf_gyr[0], lpf_gyr_alpha);
	FirstOrderIIR_Init(&filt->lpf_gyr[1], lpf_gyr_alpha);
	FirstOrderIIR_Init(&filt->lpf_gyr[2], lpf_gyr_alpha);
}

void CompFilterRollPitch_Update(CompFilterRollPitch *filt, float *acc_mps2, float *gyr_rps)
{
	// Filter XL and Gyro measurements
	for (uint8_t i = 0; i < 3; i++)
	{
		FirstOrderIIR_Update(&filt->lpf_acc[i], acc_mps2[i]);
		FirstOrderIIR_Update(&filt->lpf_gyr[i], gyr_rps[i]);
	}

	float ax_mps2 = filt->lpf_acc[0].out;
	float ay_mps2 = filt->lpf_acc[1].out;
	float az_mps2 = filt->lpf_acc[2].out;
	float p_rps   = filt->lpf_gyr[0].out;
	float q_rps   = filt->lpf_gyr[1].out;
	float r_rps   = filt->lpf_gyr[2].out;

	// Estimate angles using filtered XL measurements (units cancel out)
	float roll_acc_rad  = atan2f(ay_mps2 , az_mps2);
	float pitch_acc_rad =  asinf(ax_mps2 / 1.0f);
//	float pitch_acc_rad =  asinf(ax_mps2 / G_MPS2);

	// Transform body rates to Euler rates (gyro units need to be in radians per second)
	float roll_dot_rad  = p_rps + tanf(filt->pitch_rad) * (sinf(filt->roll_rad) * q_rps + cosf(filt->roll_rad) * r_rps);
	float pitch_dot_rad = 																 cosf(filt->roll_rad) * q_rps - sinf(filt->roll_rad) * r_rps;

	// Combine XL estimates with integral of gyro estimates to get roll and pitch estimates
	float roll_rad  = 				filt->comp_alpha  *  roll_acc_rad
									+ (1.0f - filt->comp_alpha) * (filt->roll_rad  + (filt->sample_time_ms / 1000.0f) * roll_dot_rad);

	float pitch_rad = 				filt->comp_alpha  *  pitch_acc_rad
									+ (1.0f - filt->comp_alpha) * (filt->pitch_rad + (filt->sample_time_ms / 1000.0f) * pitch_dot_rad);

	filt->roll_rad  = roll_rad;
	filt->pitch_rad = pitch_rad;
}
