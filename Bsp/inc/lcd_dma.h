#ifndef __LCD_DMA_H
#define __LCD_DMA_H		
#include "main.h"
#include "lvgl.h"                // 它为整个LVGL提供了更完整的头文件引用
#include "lv_port_disp.h"        // LVGL的显示支持
#include "lv_port_indev.h"       // LVGL的触屏支持
#include "lcdfont.h"
/* DMA传输完成标志 */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;

void LCD_Fill_DMA(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);
void LCD_LVGL_Color_Fill_DMA(u16 sx, u16 sy, u16 ex, u16 ey, lv_color_t *color);
void LCD_DrawPoint_DMA(u16 x,u16 y,u16 color);
void LCD_DrawLine_DMA(u16 x1,u16 y1,u16 x2,u16 y2,u16 color);
void LCD_DrawRectangle_DMA(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);
void Draw_Circle_DMA(u16 x0,u16 y0,u8 r,u16 color);
void LCD_ShowChinese_DMA(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);
void LCD_ShowChinese12x12_DMA(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);
void LCD_ShowChinese16x16_DMA(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);
void LCD_ShowChinese24x24_DMA(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);
void LCD_ShowChinese32x32_DMA(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);
void LCD_ShowChar_DMA(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode);
void LCD_ShowString_DMA(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode);
u32 mypow_DMA(u8 m,u8 n);
void LCD_ShowIntNum_DMA(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey);
void LCD_ShowFloatNum1_DMA(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey);
void LCD_ShowPicture_DMA(u16 x,u16 y,u16 length,u16 width,const u8 pic[]);

#endif





