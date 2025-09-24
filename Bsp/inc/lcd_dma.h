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

#endif /* __LCD_DMA_H */
