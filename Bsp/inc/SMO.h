#ifndef __SMO_H__
#define __SMO_H__

#include "main.h"
#include <math.h>

void Idq_LPF_Filter(float*data);
float LPF_Filter(float data);
float Vab_LPF_Filter(float data);
float We_Filter(float data);
float Limit(float value,float lim);
void SMO(void);
void PLL_SMO(float*Vin);




#endif
