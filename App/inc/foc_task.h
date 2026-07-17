#ifndef FOC_TASK_H
#define FOC_TASK_H

#include "main.h"

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
