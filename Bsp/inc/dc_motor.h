#ifndef __DC_MOTOR_H
#define __DC_MOTOR_H
#include "main.h"

/* 直流有刷电机(第二台电机,独立于主BLDC): 正交编码器测速/测位置(TIM4 Encoder Mode,
   PD12/PD13,4倍频计数),H桥双PWM驱动(TIM3 CH1=IN1/PA6, CH2=IN2/PA7,TB6612/DRV8833式接线:
   IN1给PWM、IN2拉低=正转,IN1拉低、IN2给PWM=反转)。控制节拍复用FOC同一颗TIM10定时器
   (App/inc/foc_task.h 的 FOC_LOOP_DT_S),和BLDC互不干扰,可以同时独立运行。 */

typedef enum {
    DC_MOTOR_MODE_IDLE     = 0,
    DC_MOTOR_MODE_SPEED    = 1,
    DC_MOTOR_MODE_POSITION = 2
} DCMotor_Mode_t;

extern volatile DCMotor_Mode_t g_dc_motor_mode;
extern volatile float DCMotor_Mech_Angle;  /* 机械角,rad,归一化到[0,2pi) */
extern volatile float DCMotor_Mech_RPM;
extern volatile float DCMotor_PWM_Duty;    /* 当前占空比,[-1,1],正负表示转向,仅供遥测显示 */

/* 上电初始化: 启动TIM3双路PWM(占空比先设0)、启动TIM4编码器计数、清零角度/测速累加器。
   必须在HAL_TIM_PWM_Start/HAL_TIM_Encoder_Start可用之后调用(即MX_TIM3_Init/MX_TIM4_Init之后)。 */
void DCMotor_Init(void);

/* 供UART命令调用的模式切换入口,mode非法值会被忽略 */
void DCMotor_SetMode(uint8_t mode);

/* TIM10中断里按FOC同一节拍调用: 读编码器增量、更新角度/转速、按当前模式跑PID、输出PWM */
void DCMotor_ControlTick(void);

#endif
