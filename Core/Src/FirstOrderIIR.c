#include "FirstOrderIIR.h"

void FirstOrderIIR_Init(FirstOrderIIR *filt, float alpha)
{
	// Check filter coefficient bounds and store
	filt->alpha = alpha;
	if (alpha < 0.0f)
		filt->alpha = 0.0f;
	else if (alpha > 1.0f)
		filt->alpha = 1.0f;

	// Clear output
	filt->out = 0.0f;
}

float FirstOrderIIR_Update(FirstOrderIIR *filt, float input)
{
	// Update filter output based on last output and new input
	// Vout[n] = (1 - alpha) * Vin[n] + alpha * Vout[n - 1]
	filt->out = (1.0f - filt->alpha) * input + filt->alpha * filt->out;

	// Return new filter output
	return filt->out;
}
