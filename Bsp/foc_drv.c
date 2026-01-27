#include "math.h"
#include "main.h"
#include "foc_drv.h"
extern float Udc;
extern Key_Struct_init Key[3];
void Clarke_transform(Iabc_Struct *I_abc,Ialpbe_Struct *I_alpbe)
{
    // ??Clarke??(??????)
    I_alpbe->I_alpha = (2.0f/3.0f) * (I_abc->Ia - 0.5f * I_abc->Ib - 0.5f * I_abc->Ic);
    I_alpbe->I_beta = (2.0f/3.0f) * ((sqrtf(3.0f)/2.0f) * (I_abc->Ib - I_abc->Ic));
}

void Park_transform(Iqd_Struct *I_dq,Ialpbe_Struct *I_alpbe,float angle)
{
    // foc_I->id = foc_I->i_alpha*cos(angle) + foc_I->i_beta*sin(angle);
    // foc_I->iq = -foc_I->i_alpha*sin(angle) + foc_I->i_beta*cos(angle);

    I_dq->Id = I_alpbe->I_alpha*sin(angle) - I_alpbe->I_beta*cos(angle);
    I_dq->Iq = I_alpbe->I_alpha*cos(angle) + I_alpbe->I_beta*sin(angle);

}
//
void svpwm_init(Udq_Struct *U_dq,float Ud,float Uq)
{
  U_dq->Ud = Ud; 
  U_dq->Uq = Uq;
}

float _normalizeAngle(float angle)
{
	float a = fmod(angle, 6.283185307f); 
	return a >= 0.0f ? a : (a + 6.283185307f);
}

void inverseParkTransform(Udq_Struct *U_dq,Ualpbe_Struct *U_alphaBeta,float angle)
{
  angle = _normalizeAngle(angle);

  float cosAngle = cos(angle);
  float sinAngle = sin(angle);

  // U_alphaBeta->Ualpha = U_dq->Ud * cosAngle - U_dq->Uq * sinAngle;
  // U_alphaBeta->Ubeta  = U_dq->Ud * sinAngle + U_dq->Uq * cosAngle;
  U_alphaBeta->U_alpha = U_dq->Ud * sinAngle + U_dq->Uq * cosAngle;//my
  U_alphaBeta->U_beta = -U_dq->Ud * cosAngle + U_dq->Uq * sinAngle;//my
}

void svpwm_sector_choice(SVPWM_Struct *svpwm,Ualpbe_Struct *U_alphaBeta)  //??????
{
  u8 x,y,z;
  u8 sector_num = 0;
  svpwm->sector = 0;
  svpwm->U1 = U_alphaBeta->U_beta;
  svpwm->U2 = 0.8660254f*U_alphaBeta->U_alpha - U_alphaBeta->U_beta/2.0f;
  svpwm->U3 = -0.8660254f*U_alphaBeta->U_alpha - U_alphaBeta->U_beta/2.0f;
  if(svpwm->U1>0) x = 1;  else x = 0;
  if(svpwm->U2>0) y = 1;  else y = 0;
  if(svpwm->U3>0) z = 1;  else z = 0;
  sector_num = 4*z+2*y+x;
  switch(sector_num)
  {
    case 1:
      svpwm->sector = 2;
      break;
    case 2:
      svpwm->sector = 6;
      break;
    case 3:
      svpwm->sector = 1;
      break;
    case 4:
      svpwm->sector = 4;
      break;
    case 5:
      svpwm->sector = 3;
      break;
    case 6:
      svpwm->sector = 5;
      break;
  }
}

void SVPWM_timer_period_set(SVPWM_Struct *svpwm,Ualpbe_Struct *U_alphaBeta)
{
  float sum,k_svpwm;

  svpwm->Ts = 1;
  // svpwm->U_alpha = U_alphaBeta->U_alpha;
  // svpwm->U_beta = U_alphaBeta->U_beta;

  float K = 1.73205080756f*svpwm->Ts/Udc;
  svpwm->Ux = U_alphaBeta->U_beta*K;
  svpwm->Uy = (0.8660254f*U_alphaBeta->U_alpha-0.5f*U_alphaBeta->U_beta)*K;
  svpwm->Uz = (-0.8660254f*U_alphaBeta->U_alpha-0.5f*U_alphaBeta->U_beta)*K;

  switch(svpwm->sector)
  {
    case 1:
    svpwm->t4 = svpwm->Uy;
    svpwm->t6 = svpwm->Ux;
    sum = svpwm->t4 + svpwm->t6;
    if(sum>svpwm->Ts)
    {
      k_svpwm = svpwm->Ts/sum;
      svpwm->t4 = svpwm->t4*k_svpwm;
      svpwm->t6 = svpwm->t6*k_svpwm;
    }
    svpwm->t7 = (svpwm->Ts - svpwm->t4 - svpwm->t6)/2.0f;
    svpwm->tc = svpwm->t7;
    svpwm->tb = svpwm->tc + svpwm->t6;
    svpwm->ta = svpwm->tb + svpwm->t4;
      break;
    case 2:
    svpwm->t2 = -svpwm->Uy;
    svpwm->t6 = -svpwm->Uz;
    sum = svpwm->t2 + svpwm->t6;
    if(sum>svpwm->Ts)
    {
      k_svpwm = svpwm->Ts/sum;
      svpwm->t2 = svpwm->t2*k_svpwm;
      svpwm->t6 = svpwm->t6*k_svpwm;
    }
    svpwm->t7 = (svpwm->Ts - svpwm->t2 - svpwm->t6)/2.0f;
    svpwm->tc = svpwm->t7;
    svpwm->tb = svpwm->t2 + svpwm->t6 + svpwm->t7;
    svpwm->ta = svpwm->t6 + svpwm->t7;
      break;
    case 3:
    svpwm->t2 = svpwm->Ux;
    svpwm->t3 = svpwm->Uz;
    sum = svpwm->t2 + svpwm->t3;
    if(sum>svpwm->Ts)
    {
      k_svpwm = svpwm->Ts/sum;
      svpwm->t2 = svpwm->t2*k_svpwm;
      svpwm->t3 = svpwm->t3*k_svpwm;
    }
    svpwm->t7 = (svpwm->Ts - svpwm->t2 - svpwm->t3)/2.0f;
    svpwm->tc = svpwm->t7 + svpwm->t3;
    svpwm->tb = svpwm->t2 + svpwm->t3 + svpwm->t7;
    svpwm->ta = svpwm->t7;
      break;
    case 4:
    svpwm->t1 = -svpwm->Ux;
    svpwm->t3 = -svpwm->Uy;
    sum = svpwm->t1 + svpwm->t3;
    if(sum>svpwm->Ts)
    {
      k_svpwm = svpwm->Ts/sum;
      svpwm->t1 = svpwm->t1*k_svpwm;
      svpwm->t3 = svpwm->t3*k_svpwm;
    }
    svpwm->t7 = (svpwm->Ts - svpwm->t1 - svpwm->t3)/2.0f;
    svpwm->tc = svpwm->t7 + svpwm->t1 + svpwm->t3;
    svpwm->tb = svpwm->t3 + svpwm->t7;
    svpwm->ta = svpwm->t7;
      break;      
    case 5:
    svpwm->t1 = svpwm->Uz;
    svpwm->t5 = svpwm->Uy;
    sum = svpwm->t1 + svpwm->t5;
    if(sum>svpwm->Ts)
    {
      k_svpwm = svpwm->Ts/sum;
      svpwm->t1 = svpwm->t1*k_svpwm;
      svpwm->t5 = svpwm->t5*k_svpwm;
    }
    svpwm->t7 = (svpwm->Ts - svpwm->t1 - svpwm->t5)/2.0f;
    svpwm->tc = svpwm->t7 + svpwm->t1 + svpwm->t5;
    svpwm->tb = svpwm->t7;
    svpwm->ta = svpwm->t7 + svpwm->t5;
      break;   
    case 6:
    svpwm->t4 = -svpwm->Uz;
    svpwm->t5 = -svpwm->Ux;
    sum = svpwm->t4 + svpwm->t5;
    if(sum>svpwm->Ts)
    {
      k_svpwm = svpwm->Ts/sum;
      svpwm->t4 = svpwm->t4*k_svpwm;
      svpwm->t5 = svpwm->t5*k_svpwm;
    }
    svpwm->t7 = (svpwm->Ts - svpwm->t4 - svpwm->t5)/2.0f;
    svpwm->tc = svpwm->t7 + svpwm->t5;
    svpwm->tb = svpwm->t7;
    svpwm->ta = svpwm->t7 + svpwm->t5 + svpwm->t4;
      break; 
  }
}

void  Key_read(void)
{
    if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
        Key[0].Key_State =  0;
    }
    else  if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET)
    {
        Key[0].Key_State =  1;
    }
    if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
    {
        Key[1].Key_State =  0;
    }
    else  if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET)
    {
        Key[1].Key_State =  1;
    }
    if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET)
    {
        Key[2].Key_State =  0;
    }
    else  if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_SET)
    {
        Key[2].Key_State =  1;
    }
}
