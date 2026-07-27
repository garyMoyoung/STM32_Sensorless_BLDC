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

// 位置式PID, 积分/微分项显式按dt折算成标准的"每秒"物理量,
// 这样调用频率(控制环节拍)变化时不需要重新折算Ki/Kd背后的隐含时间步长
float PID_Position_Calculate(PIDController *pid, float target, float current, float dt)
{
    float derivative;

    // 计算误差
    pid->error = target - current;

    // 积分项按真实时间累加(而不是每调用一次就加一次error)
    pid->integral += pid->error * dt;

    // 积分限幅(抗积分饱和)
    if(pid->integral > pid->maxIntegral) {
        pid->integral = pid->maxIntegral;
    }
    else if(pid->integral < -pid->maxIntegral) {
        pid->integral = -pid->maxIntegral;
    }

    // 微分项按真实时间做差分, dt<=0时视为无效调用,微分项直接置0避免除0
    derivative = (dt > 0.0f) ? ((pid->error - pid->lastError) / dt) : 0.0f;

    // 计算PID输出
    pid->output = pid->kp * pid->error +
                 pid->ki * pid->integral +
                 pid->kd * derivative;

    // 输出限幅
    if(pid->output >= pid->maxOutput) {
        pid->output = pid->maxOutput;
    }
    else if(pid->output <= pid->minOutput) {
        pid->output = pid->minOutput;
    }
    // 保存本次误差供下次微分计算
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

// 模式切换时清空积分/误差历史,不改增益/限幅/目标值,避免残留积分导致输出突跳
void PID_Reset(PIDController *pid)
{
    pid->error = 0;
    pid->lastError = 0;
    pid->preError = 0;
    pid->integral = 0;
    pid->output = 0;
}

float First_order_Filtering(float input)
{
  float output;
  static float output_last;
  output = 0.3f*output_last + 0.7f*input;
  output_last = output;
  return output;
}
