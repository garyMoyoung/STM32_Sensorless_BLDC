#ifndef __PID_H
#define __PID_H
#include "math.h"

float PID_Position_Calculate(PIDController *pid, float target, float current);
float PID_Increment_Calculate(PIDController *pid, float target, float current);
float First_order_Filtering(float input);
void PID_Init(PIDController *pid,float maxOutput, float minOutput, float maxIntegral);
void PID_param_set(PIDController *pid, float kp, float ki, float kd);
#endif

