/*
 * FirstOrderIIR.h
 *
 * Author: Kyle Erf
 * Created: Jan 16 2023
 *
 */

#ifndef INC_FIRSTORDERIIR_H_
#define INC_FIRSTORDERIIR_H_

typedef struct
{
	float alpha;
	float out;
} FirstOrderIIR;

void FirstOrderIIR_Init(FirstOrderIIR *filt, float alpha);
float FirstOrderIIR_Update(FirstOrderIIR *filt, float input);

#endif /* INC_FIRSTORDERIIR_H_ */
