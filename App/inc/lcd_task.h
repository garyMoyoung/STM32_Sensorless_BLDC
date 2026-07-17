#ifndef __LCD_TASK_H__
#define __LCD_TASK_H__

#include "main.h"

/* LCD 遥测显示开关(运行时可通过UART切换), 默认开启 */
extern volatile uint8_t g_lcd_enable;

/* 供UART命令(0x93 SET_LCD_ENABLE / ASCII $LCD,n#)调用 */
void LCD_SetEnable(uint8_t enable);

void LcdTask_Entry(void *argument);

#endif
