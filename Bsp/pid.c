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
    float p_term;
    float d_term;
    float unsat_output;
    float integral_candidate;

    // 计算误差
    pid->error = target - current;

    // 微分项按真实时间做差分, dt<=0时视为无效调用,微分项直接置0避免除0
    derivative = (dt > 0.0f) ? ((pid->error - pid->lastError) / dt) : 0.0f;

    p_term = pid->kp * pid->error;
    d_term = pid->kd * derivative;

    /* 抗积分饱和(条件积分法): 先按"若本次积分正常累加"试算integral和output,
       如果试算output已经超出[minOutput,maxOutput]、且继续累加积分只会让饱和更严重
       (即误差方向与当前饱和方向一致),就冻结积分不再累加,避免output被输出限幅钳死期间
       integral还在继续无意义地增长、之后要花很久才能"退出饱和"导致的大幅超调/来回跳变。
       之前只做integral本身限幅(maxIntegral)是不够的: 只要maxIntegral设得比
       maxOutput/ki大(本工程electric流环maxIntegral=100远超±4V/ki),integral在触及
       输出限幅前就已经能让ki*integral本身超过maxOutput,等于没起到抗饱和作用。 */
    integral_candidate = pid->integral + pid->error * dt;
    unsat_output = p_term + pid->ki * integral_candidate + d_term;

    if ((unsat_output > pid->maxOutput) && (pid->error > 0.0f))
    {
        /* 已上限饱和且误差还在往同方向推,冻结积分 */
    }
    else if ((unsat_output < pid->minOutput) && (pid->error < 0.0f))
    {
        /* 已下限饱和且误差还在往同方向推,冻结积分 */
    }
    else
    {
        pid->integral = integral_candidate;
    }

    // 积分限幅(兜底,防止kp/kd为0等极端参数下integral本身失控增长)
    if(pid->integral > pid->maxIntegral) {
        pid->integral = pid->maxIntegral;
    }
    else if(pid->integral < -pid->maxIntegral) {
        pid->integral = -pid->maxIntegral;
    }

    // 计算PID输出
    pid->output = p_term + pid->ki * pid->integral + d_term;

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
