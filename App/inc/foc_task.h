#ifndef FOC_TASK_H
#define FOC_TASK_H

#include "main.h"

/* TIM10实测: 168MHz/(84分频)=2MHz计数时钟, ARR=1000 => 更新频率约2kHz(约0.5ms/拍)。
   该值同时喂给BLDC侧PID的dt形参、Mech_RPM/开环电角度积分,以及Bsp/dc_motor.c(直流电机控制,
   同样挂在TIM10这拍里)的dt形参和测速积分,以后如果改了TIM10的Prescaler/Period,
   这一处改了两边都跟着变,不用分别改。 */
#define FOC_LOOP_DT_S                    0.0005f

/* FOC 运行模式(运行时可通过UART切换), 上电默认IDLE(PWM零输出), 避免上电即自动转动 */
extern volatile FOC_Mode_t g_foc_mode;

/* 开环调试参数, 默认值见 foc_task.c, 可通过UART在线调节(见 uart_task.c ProcessDataFrame case 0x06~0x08) */
extern float OpenLoop_AlignUd_V;
extern float OpenLoop_RunUq_V;
extern float OpenLoop_TargetElecHz;

void angle_proc(void);
void Queue_proc(void);

/* 1kHz(TIM10)调用入口: 按 g_foc_mode 分发到 IDLE/开环/电流环/速度环/位置环 */
void FOC_ModeDispatch(void);

/* 供UART命令(0x90 SET_MODE / 0x91 DISARM)调用的模式切换入口 */
void FOC_SetMode(uint8_t mode);

/* 供UART命令(0x87 RECALIBRATE)调用的重新标零入口: 1=已执行标定, 0=非IDLE模式,拒绝执行 */
uint8_t FOC_RequestRecalibration(void);

void AngleTask_Entry(void *argument);

#endif
