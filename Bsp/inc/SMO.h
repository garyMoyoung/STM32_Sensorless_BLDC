#ifndef __SMO_H__
#define __SMO_H__

#include "main.h"
#include <math.h>

/* 电机极对数,必须和 App/foc_task.c 里 Elec_Angle = 7.0f * Mech_Angle 保持一致,
   否则SMO内部电角速度<->机械转速的换算和实际电角度对不上。 */
#define MOTOR_POLE_PAIRS 7.0f

void Idq_LPF_Filter(float*data);
float LPF_Filter(float data, uint8_t ch);
float Vab_LPF_Filter(float data, uint8_t ch);
float We_Filter(float data);
float Limit(float value,float lim);
void SMO(void);
void PLL_SMO(float*Vin);

/* 影子模式接入口: 用真实测得的相电流/施加电压(alpha-beta,来自Clarke变换和SVPWM的
   逆Park结果)和真实电角度(AS5600测得)喂给SMO,只做估算、不参与PID控制输出。
   方便先拿真实角度核对SMO自己估的Theta_fore_New/We_fore准不准,再决定要不要真正
   切到无感闭环。*/
void SMO_ShadowUpdate(float i_alpha, float i_beta, float u_alpha, float u_beta, float elec_angle);

#endif
