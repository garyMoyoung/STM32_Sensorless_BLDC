#include "lcd_task.h"
#include "cmsis_os.h"
#include "lcd_init.h"
#include "lcd.h"
#include "foc_task.h"
#include <stdio.h>
#include <string.h>

extern volatile float Mech_Angle;
extern volatile float Mech_RPM;
extern Iabc_Struct Iabc_M0;

/* LCD 遥测显示开关, 默认开启(见 lcd_task.h)。上位机通过 0x93 SET_LCD_ENABLE / ASCII $LCD,n# 切换 */
volatile uint8_t g_lcd_enable = 1U;

#define LCD_REFRESH_PERIOD_MS 150U

#define ROW_TITLE  10U
#define ROW_MODE   40U
#define ROW_RPM    65U
#define ROW_ANGLE  90U
#define ROW_IA     115U
#define ROW_IB     140U
#define ROW_IC     165U

#define COL_LABEL  10U
#define COL_VALUE  100U

static const char *ModeName(FOC_Mode_t mode)
{
    switch (mode)
    {
        case FOC_MODE_IDLE:      return "IDLE";
        case FOC_MODE_OPEN_LOOP: return "OPEN";
        case FOC_MODE_CURRENT:   return "CUR";
        case FOC_MODE_SPEED:     return "SPD";
        case FOC_MODE_POSITION:  return "POS";
        default:                 return "?";
    }
}

/* 只在开机/重新开启时画一次的静态标签,数值区域(COL_VALUE起)由 LCD_RefreshValues 周期刷新 */
static void LCD_DrawStaticLayout(void)
{
    LCD_Fill_DMA(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString_DMA(COL_LABEL, ROW_TITLE, (const u8 *)"FOC Monitor", CYAN, BLACK, 16, 0);
    LCD_ShowString_DMA(COL_LABEL, ROW_MODE,  (const u8 *)"Mode:",   WHITE, BLACK, 16, 0);
    LCD_ShowString_DMA(COL_LABEL, ROW_RPM,   (const u8 *)"RPM:",    WHITE, BLACK, 16, 0);
    LCD_ShowString_DMA(COL_LABEL, ROW_ANGLE, (const u8 *)"Ang(deg):", WHITE, BLACK, 16, 0);
    LCD_ShowString_DMA(COL_LABEL, ROW_IA,    (const u8 *)"Ia(A):",  WHITE, BLACK, 16, 0);
    LCD_ShowString_DMA(COL_LABEL, ROW_IB,    (const u8 *)"Ib(A):",  WHITE, BLACK, 16, 0);
    LCD_ShowString_DMA(COL_LABEL, ROW_IC,    (const u8 *)"Ic(A):",  WHITE, BLACK, 16, 0);
}

/* 数值区域固定宽度格式化后整段覆盖重绘,避免新旧数字位数不同残留字符 */
static void LCD_RefreshValues(void)
{
    char buf[16];
    FOC_Mode_t mode = g_foc_mode;
    float rpm = Mech_RPM;
    float angle_deg = Mech_Angle * (180.0f / 3.14159265f);
    float ia = Iabc_M0.Ia;
    float ib = Iabc_M0.Ib;
    float ic = Iabc_M0.Ic;

    snprintf(buf, sizeof(buf), "%-6s", ModeName(mode));
    LCD_ShowString_DMA(COL_VALUE, ROW_MODE, (const u8 *)buf, YELLOW, BLACK, 16, 0);

    snprintf(buf, sizeof(buf), "%7.1f", rpm);
    LCD_ShowString_DMA(COL_VALUE, ROW_RPM, (const u8 *)buf, YELLOW, BLACK, 16, 0);

    snprintf(buf, sizeof(buf), "%7.1f", angle_deg);
    LCD_ShowString_DMA(COL_VALUE, ROW_ANGLE, (const u8 *)buf, YELLOW, BLACK, 16, 0);

    snprintf(buf, sizeof(buf), "%7.2f", ia);
    LCD_ShowString_DMA(COL_VALUE, ROW_IA, (const u8 *)buf, YELLOW, BLACK, 16, 0);

    snprintf(buf, sizeof(buf), "%7.2f", ib);
    LCD_ShowString_DMA(COL_VALUE, ROW_IB, (const u8 *)buf, YELLOW, BLACK, 16, 0);

    snprintf(buf, sizeof(buf), "%7.2f", ic);
    LCD_ShowString_DMA(COL_VALUE, ROW_IC, (const u8 *)buf, YELLOW, BLACK, 16, 0);
}

void LCD_SetEnable(uint8_t enable)
{
    g_lcd_enable = (enable != 0U) ? 1U : 0U;
}

void LcdTask_Entry(void *argument)
{
    uint8_t prev_enable = 0xFFU; /* 强制第一次循环触发一次状态处理 */

    LCD_Init_DMA();
    LCD_BLK_Clr();
    LCD_Fill_DMA(0, 0, LCD_W, LCD_H, BLACK);

    for (;;)
    {
        uint8_t enable_now = g_lcd_enable;

        if (enable_now != prev_enable)
        {
            if (enable_now != 0U)
            {
                LCD_DrawStaticLayout();
                LCD_BLK_Set();
            }
            else
            {
                LCD_BLK_Clr();
            }
            prev_enable = enable_now;
        }

        if (enable_now != 0U)
        {
            LCD_RefreshValues();
        }

        osDelay(LCD_REFRESH_PERIOD_MS);
    }
}
