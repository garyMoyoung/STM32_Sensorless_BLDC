#ifndef __LVGL_DEMO_TASK_H__
#define __LVGL_DEMO_TASK_H__

#include "main.h"

/* LVGL 测试动画开关(运行时可通过UART切换), 默认关闭, 不影响现有LCD遥测显示 */
extern volatile uint8_t g_lvgl_demo_enable;

/* 供UART命令(0x94 SET_LVGL_ENABLE / ASCII $LVGL,n#)调用 */
void LvglDemo_SetEnable(uint8_t enable);

/* 由 LvglTimerTask 每个周期(5ms)调用一次:
   负责在开关状态变化时与LCD遥测任务(g_lcd_enable)互斥切换LCD/SPI总线的使用权,
   开关为1时驱动LVGL动画场景(lv_tick_inc/lv_timer_handler)并统计DMA刷新帧率 */
void LvglDemo_Process(void);

#endif
