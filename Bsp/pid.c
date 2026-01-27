#include "math.h"
#include "stdio.h"
#include "stdarg.h"
#include <stdbool.h>
#include <stdlib.h>
#include "main.h"
#include <string.h>
#include "stdlib.h"
#include "foc_drv.h"
#include "stdarg.h"
#include <string.h>
#include "pid.h"

// ???PID??
float PID_Position_Calculate(PIDController *pid, float target, float current) 
{
    // ????
    pid->error = target - current;
    
    // ?????
    pid->integral += pid->error;

    // ????
    if(pid->integral > pid->maxIntegral) {
        pid->integral = pid->maxIntegral;
    }
    else if(pid->integral < -pid->maxIntegral) {
        pid->integral = -pid->maxIntegral;
    }
    // ???PID??
    pid->output = pid->kp * pid->error +
                 pid->ki * pid->integral +
                 pid->kd * (pid->error - pid->lastError);
    
    // ????
    if(pid->output >= pid->maxOutput) {
        pid->output = pid->maxOutput;
    }
    else if(pid->output <= pid->minOutput) {
        pid->output = pid->minOutput;
    }
    // ????
    pid->lastError = pid->error;
    
    return pid->output;
}

// ???PID??
float PID_Increment_Calculate(PIDController *pid, float target, float current) 
{
    float increment = 0;  // PID???
    
    // ????
    pid->error = target - current;
    
    // ???PID??
    increment = pid->kp * (pid->error - pid->lastError) +
               pid->ki * pid->error +
               pid->kd * (pid->error - 2*pid->lastError + pid->preError);

    // ????
    if(pid->output > pid->maxOutput) {
        pid->output = pid->maxOutput;
    }
    else if(pid->output < pid->minOutput) {
        pid->output = pid->minOutput;
    }
    // ?????
    pid->output += increment;
    // ????
    pid->preError = pid->lastError;
    pid->lastError = pid->error;
    return pid->output;
}

void PID_param_set(PIDController *pid, float kp, float ki, float kd) 
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_Init(PIDController *pid,float maxOutput, float minOutput, float maxIntegral) {
    pid->error = 0;
    pid->lastError = 0;
    pid->preError = 0;
    pid->integral = 0;
    pid->output = 0;
    
    // ????PID??
    pid->kp = 0;
    pid->ki = 0;
    pid->kd = 0;
    
    // ???????
    pid->maxOutput = maxOutput;
    pid->minOutput = minOutput;
    pid->maxIntegral = maxIntegral;
}

float First_order_Filtering(float input)
{
  float output;
  static float output_last;
  output = 0.3f*output_last + 0.7f*input;
  output_last = output;
  return output;
}
