#ifndef __PID_H
#define __PID_H
#include "math.h"

/* dt: 本次调用相对上次调用的真实时间间隔(秒), 积分项按 error*dt 累加、微分项按 (error-lastError)/dt
   计算, 使Ki/Kd是标准的"每秒"物理量, 不再随调用频率隐式变化 */
float PID_Position_Calculate(PIDController *pid, float target, float current, float dt);
float PID_Increment_Calculate(PIDController *pid, float target, float current);
float First_order_Filtering(float input);
void PID_Init(PIDController *pid,float maxOutput, float minOutput, float maxIntegral);
void PID_param_set(PIDController *pid, float kp, float ki, float kd);
void PID_Reset(PIDController *pid);
#endif

