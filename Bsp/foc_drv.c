#include "math.h"
#include "main.h"
#include "foc_drv.h"
#include "arm_math.h"

extern float Udc;
extern Key_Struct_init Key[3];

/**
 * @brief Clarke变换 - 直接计算（无需矩阵库）
 * @param I_abc: 三相电流输入
 * @param I_alpbe: α-β坐标系输出
 */
void Clarke_transform(Iabc_Struct *I_abc, Ialpbe_Struct *I_alpbe)
{
    // Clarke变换公式
    // I_alpha = Ia
    // I_beta = (Ia + 2*Ib) / √3 = (Ia + 2*Ib) * sqrt(3)/3
    
    float32_t sqrt3_inv = 0.57735026919f;  // 1/√3
    
    I_alpbe->I_alpha = I_abc->Ia;
    I_alpbe->I_beta = (I_abc->Ia + 2.0f * I_abc->Ib) * sqrt3_inv;
}

/**
 * @brief Park变换 - 使用快速三角函数
 * @param I_dq: dq轴电流输出
 * @param I_alpbe: α-β坐标系输入
 * @param angle: 电气角度 (弧度)
 */
void Park_transform(Iqd_Struct *I_dq, Ialpbe_Struct *I_alpbe, float angle)
{
    float32_t cos_val, sin_val;
    
    // 使用ARM DSP库的快速三角函数（返回值方式）
    sin_val = arm_sin_f32(angle);
    cos_val = arm_cos_f32(angle);
    
    // Park变换矩阵乘法
    // [Id]   [cos(θ)   sin(θ) ] [I_alpha]
    // [Iq] = [-sin(θ)  cos(θ)] [I_beta ]
    
    I_dq->Id = I_alpbe->I_alpha * cos_val + I_alpbe->I_beta * sin_val;
    I_dq->Iq = -I_alpbe->I_alpha * sin_val + I_alpbe->I_beta * cos_val;
}

/**
 * @brief 逆Park变换 - 使用快速三角函数
 * @param U_dq: dq轴电压输入
 * @param U_alphaBeta: α-β坐标系输出
 * @param angle: 电气角度 (弧度)
 */
void inverseParkTransform(Udq_Struct *U_dq, Ualpbe_Struct *U_alphaBeta, float angle)
{
    float32_t cos_val, sin_val;
    
    // 使用ARM DSP库的快速三角函数（返回值方式）
    sin_val = arm_sin_f32(angle);
    cos_val = arm_cos_f32(angle);
    
    // 逆Park变换
    // [U_alpha]   [cos(θ)  -sin(θ)] [Ud]
    // [U_beta ] = [sin(θ)   cos(θ)] [Uq]
    
    U_alphaBeta->U_alpha = U_dq->Ud * cos_val - U_dq->Uq * sin_val;
    U_alphaBeta->U_beta = U_dq->Ud * sin_val + U_dq->Uq * cos_val;
}

/**
 * @brief SVPWM扇区选择
 * @param svpwm: SVPWM结构体
 * @param U_alphaBeta: α-β电压
 */
void svpwm_sector_choice(SVPWM_Struct *svpwm, Ualpbe_Struct *U_alphaBeta)
{
    uint8_t x, y, z;
    uint8_t sector_num = 0;
    
    svpwm->sector = 0;
    svpwm->U1 = U_alphaBeta->U_beta;
    svpwm->U2 = 0.8660254f * U_alphaBeta->U_alpha - U_alphaBeta->U_beta / 2.0f;
    svpwm->U3 = -0.8660254f * U_alphaBeta->U_alpha - U_alphaBeta->U_beta / 2.0f;
    
    // 判断符号
    x = (svpwm->U1 > 0) ? 1 : 0;
    y = (svpwm->U2 > 0) ? 1 : 0;
    z = (svpwm->U3 > 0) ? 1 : 0;
    
    sector_num = 4 * z + 2 * y + x;
    svpwm->sector = sector_num;
}

/**
 * @brief SVPWM定时器周期设置
 * @param svpwm: SVPWM结构体
 * @param U_alphaBeta: α-β电压
 */
void SVPWM_timer_period_set(SVPWM_Struct *svpwm, Ualpbe_Struct *U_alphaBeta)
{
    float32_t sum, k_svpwm;

    svpwm->Ts = 1.0f;
    
    // 使用常数而不是计算
    float32_t K = 1.73205080756f * svpwm->Ts / Udc;  // √3 * Ts / Udc
    
    // 计算三个方向的电压分量
    svpwm->Ux = U_alphaBeta->U_beta * K;
    svpwm->Uy = (0.8660254f * U_alphaBeta->U_alpha + 0.5f * U_alphaBeta->U_beta) * K;
    svpwm->Uz = (-0.8660254f * U_alphaBeta->U_alpha + 0.5f * U_alphaBeta->U_beta) * K;

    // 扇区判断与时间计算
    switch (svpwm->sector)
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
    // 过调制处理
    if (sum > svpwm->Ts)
    {
        k_svpwm = svpwm->Ts / sum;
        svpwm->t1 *= k_svpwm;
        svpwm->t2 = svpwm->Ts - svpwm->t1;
    }
    
    // 计算PWM占空比
    svpwm->ta = (svpwm->Ts - svpwm->t1 - svpwm->t2) / 4.0f;
    svpwm->tb = svpwm->ta + svpwm->t1 / 2.0f;//svpwm->ta + svpwm->t1 / 2.0f
    svpwm->tc = svpwm->tb + svpwm->t2 / 2.0f;//svpwm->tb + svpwm->t2 / 2.0f

    switch(svpwm->sector)
    {
      case 1:
          svpwm->tcm1 = svpwm->tb;
          svpwm->tcm2 = svpwm->ta;
          svpwm->tcm3 = svpwm->tc;
          break;
      case 2:
          svpwm->tcm1 = svpwm->ta;
          svpwm->tcm2 = svpwm->tc;
          svpwm->tcm3 = svpwm->tb;
          break;
      case 3:
          svpwm->tcm1 = svpwm->ta;
          svpwm->tcm2 = svpwm->tb;
          svpwm->tcm3 = svpwm->tc;
          break;
      case 4:
          svpwm->tcm1 = svpwm->tc;
          svpwm->tcm2 = svpwm->tb;
          svpwm->tcm3 = svpwm->ta;
          break;
      case 5:
          svpwm->tcm1 = svpwm->tc;
          svpwm->tcm2 = svpwm->ta;
          svpwm->tcm3 = svpwm->tb;
          break;
      case 6:
          svpwm->tcm1 = svpwm->tb;
          svpwm->tcm2 = svpwm->tc;
          svpwm->tcm3 = svpwm->ta;
          break;
    }
    svpwm->tcm1 = 2.0f * svpwm->tcm1;
    svpwm->tcm2 = 2.0f * svpwm->tcm2;
    svpwm->tcm3 = 2.0f * svpwm->tcm3;
}
void SVPWM(float Ele_angle, Ualpbe_Struct *Ualpbe_M0, SVPWM_Struct *SVPWM_M0, Udq_Struct *Udq_M0)
{
    inverseParkTransform(Udq_M0,Ualpbe_M0,Ele_angle);
    svpwm_sector_choice(SVPWM_M0,Ualpbe_M0);
    SVPWM_timer_period_set(SVPWM_M0,Ualpbe_M0);
}
float IF_ang_ZZ(float angle,float add_index)
{
    angle += add_index;
    if(angle > 6.2831853f) angle = 0.0f;
    return angle;
}
/** 
 * @brief SVPWM初始化
 * @param U_dq: dq轴电压
 * @param Ud: d轴电压初值
 * @param Uq: q轴电压初值
 */
void svpwm_init(Udq_Struct *U_dq, float Ud, float Uq)
{
    U_dq->Ud = Ud;
    U_dq->Uq = Uq;
}

/**
 * @brief 角度规范化 (0 ~ 2π)
 * @param angle: 输入角度
 * @retval 规范化后的角度
 */
float _normalizeAngle(float angle)
{
    float a = fmod(angle, 6.283185307f);
    return a >= 0.0f ? a : (a + 6.283185307f);
}

/**
 * @brief 按键读取
 */
void Key_read(void)
{
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
        Key[0].Key_State = 0;
    }
    else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET)
    {
        Key[0].Key_State = 1;
    }
    
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)
    {
        Key[1].Key_State = 0;
    }
    else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET)
    {
        Key[1].Key_State = 1;
    }
    
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET)
    {
        Key[2].Key_State = 0;
    }
    else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_SET)
    {
        Key[2].Key_State = 1;
    }
}