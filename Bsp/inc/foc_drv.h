#ifndef __FOC_DRV_H
#define __FOC_DRV_H

#include "main.h"



void Clarke_transform(Iabc_Struct *I_abc,Ialpbe_Struct *I_alpbe);
void Park_transform(Iqd_Struct *I_dq,Ialpbe_Struct *I_alpbe,float angle);
void svpwm_init(Udq_Struct *U_dq,float Ud,float Uq);
float _normalizeAngle(float angle);
void inverseParkTransform(Udq_Struct *U_dq,Ualpbe_Struct *U_alphaBeta,float angle);
void svpwm_sector_choice(SVPWM_Struct *svpwm,Ualpbe_Struct *U_alphaBeta);
void SVPWM_timer_period_set(SVPWM_Struct *svpwm,Ualpbe_Struct *U_alphaBeta);


#endif
