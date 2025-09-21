#ifndef __LCD_DMA_H
#define __LCD_DMA_H

#include "main.h"
#include "lvgl.h"
#include "spi.h"

/* DMA传输状态枚举 */
typedef enum {
    LCD_DMA_IDLE = 0,
    LCD_DMA_BUSY,
    LCD_DMA_COMPLETE,
    LCD_DMA_ERROR
} LCD_DMA_State_t;

/* DMA传输完成回调函数类型 */
typedef void (*LCD_DMA_Callback_t)(void);

/* DMA驱动结构体 */
typedef struct {
    LCD_DMA_State_t state;
    LCD_DMA_Callback_t callback;
    uint32_t transfer_size;
    uint8_t *data_ptr;
} LCD_DMA_Handle_t;

/* 宏定义 */
#define LCD_DMA_MAX_BUFFER_SIZE     4096    // 最大DMA缓冲区大小
#define LCD_DMA_TIMEOUT_MS          1000    // DMA超时时间

/* 函数声明 */
void LCD_DMA_Init(void);
LCD_DMA_State_t LCD_DMA_GetState(void);
HAL_StatusTypeDef LCD_DMA_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
HAL_StatusTypeDef LCD_DMA_FillArea(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *color_data);
HAL_StatusTypeDef LCD_DMA_LVGL_Fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, lv_color_t *color);
HAL_StatusTypeDef LCD_DMA_LVGL_Fill_Async(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, lv_color_t *color, LCD_DMA_Callback_t callback);
HAL_StatusTypeDef LCD_DMA_DrawBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *bitmap);
void LCD_DMA_SetCallback(LCD_DMA_Callback_t callback);
HAL_StatusTypeDef LCD_DMA_WaitForComplete(uint32_t timeout_ms);

#endif /* __LCD_DMA_H */