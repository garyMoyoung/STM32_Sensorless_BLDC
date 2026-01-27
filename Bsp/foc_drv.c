#include "math.h"
#include "main.h"
#include "foc_drv.h"
extern float Udc;
extern Key_Struct_init Key[3];
void Clarke_transform(Iabc_Struct *I_abc,Ialpbe_Struct *I_alpbe)
{
    I_alpbe->I_alpha = (2.0f/3.0f) * (I_abc->Ia - 0.5f * I_abc->Ib - 0.5f * I_abc->Ic);
    I_alpbe->I_beta = (2.0f/3.0f) * ((sqrtf(3.0f)/2.0f) * (I_abc->Ib - I_abc->Ic));
}

void Park_transform(Iqd_Struct *I_dq,Ialpbe_Struct *I_alpbe,float angle)
{
    I_dq->Id = I_alpbe->I_alpha*cos(angle) + I_alpbe->I_beta*sin(angle);
    I_dq->Iq = -I_alpbe->I_alpha*sin(angle) + I_alpbe->I_beta*cos(angle);
}

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
  U_alphaBeta->U_alpha = U_dq->Ud * cos(angle) - U_dq->Uq * sin(angle);
  U_alphaBeta->U_beta  = U_dq->Ud * sin(angle) + U_dq->Uq * cos(angle);
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
  svpwm->sector = sector_num;
}

void SVPWM_timer_period_set(SVPWM_Struct *svpwm,Ualpbe_Struct *U_alphaBeta)
{
  float sum,k_svpwm;
  svpwm->Ts = 1;
  float K = 1.73205080756f*svpwm->Ts/Udc;
  svpwm->Ux = U_alphaBeta->U_beta*K;
  svpwm->Uy = (0.8660254f*U_alphaBeta->U_alpha + 0.5f*U_alphaBeta->U_beta)*K;// -0.5f*U_alphaBeta->U_beta
  svpwm->Uz = (-0.8660254f*U_alphaBeta->U_alpha + 0.5f*U_alphaBeta->U_beta)*K;//-0.5f*U_alphaBeta->U_beta

  switch(svpwm->sector)
  {
    case 1:
      svpwm->t1 = svpwm->Uz;
      svpwm->t2 = svpwm->Uy;
      break;
    case 2:
      svpwm->t1 = svpwm->Uy;
      svpwm->t2 = -svpwm->Ux;
      break;
    case 3:
      svpwm->t1 = -svpwm->Uz;
      svpwm->t2 = svpwm->Ux;
      break;
    case 4:
      svpwm->t1 = -svpwm->Ux;
      svpwm->t2 = svpwm->Uz;
      break;      
    case 5:
      svpwm->t1 = svpwm->Ux;
      svpwm->t2 = -svpwm->Uy;
      break;   
    case 6:
      svpwm->t1 = -svpwm->Uy;
      svpwm->t2 = -svpwm->Uz;
      break; 
    default:
      break;
  }
  sum = svpwm->t1 + svpwm->t2;
  if(svpwm->t1 + svpwm->t2 > svpwm->Ts)
  {
    k_svpwm = svpwm->Ts/(svpwm->t1 + svpwm->t2);
    svpwm->t1 = svpwm->t1*k_svpwm;
    svpwm->t2 = svpwm->t2*k_svpwm;
  }
  svpwm->ta = (svpwm->Ts - svpwm->t1 - svpwm->t2)/4.0f;
  svpwm->tb = svpwm->ta + svpwm->t1/2.0f;
  svpwm->tc = svpwm->tb + svpwm->t2/2.0f;
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
