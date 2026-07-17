#ifndef __FOC_DRV_H
#define __FOC_DRV_H

#include "main.h"

/* TIM2(M0三相PWM)计数周期,中心对齐模式,对应 tim.c 里 htim2.Init.Period=3360 */
#define FOC_PWM_PERIOD 3360.0f

void Clarke_transform(Iabc_Struct *I_abc,Ialpbe_Struct *I_alpbe);
void Park_transform(Iqd_Struct *I_dq,Ialpbe_Struct *I_alpbe,float angle);
void svpwm_init(Udq_Struct *U_dq,float Ud,float Uq);
float _normalizeAngle(float angle);
float AngleErrorWrap(float error);
void inverseParkTransform(Udq_Struct *U_dq,Ualpbe_Struct *U_alphaBeta,float angle);
void svpwm_sector_choice(SVPWM_Struct *svpwm,Ualpbe_Struct *U_alphaBeta);
void SVPWM_timer_period_set(SVPWM_Struct *svpwm,Ualpbe_Struct *U_alphaBeta);
void SVPWM(float Ele_angle, Ualpbe_Struct *Ualpbe_M0, SVPWM_Struct *SVPWM_M0, Udq_Struct *Udq_M0);
float IF_ang_ZZ(float angle,float add_index);
void  Key_read(void);
void PWM_TIM2_Set(uint16_t pwm_a,uint16_t pwm_b,uint16_t pwm_c);
uint16_t PWM_LimitCompare(float compare);

#endif
