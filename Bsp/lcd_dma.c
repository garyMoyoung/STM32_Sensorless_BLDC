#include "lcd_dma.h"
#include "lcd.h"
#include "lcd_init.h"
#include <string.h>

/* 私有变量 */
static LCD_DMA_Handle_t lcd_dma_handle;
static uint16_t dma_color_buffer[LCD_DMA_MAX_BUFFER_SIZE/2]; // 颜色缓冲区
static volatile uint8_t dma_transfer_complete = 1;

/* 外部变量 */
extern SPI_HandleTypeDef hspi2;  // 假设使用SPI2连接LCD

/**
 * @brief  初始化LCD DMA驱动
 * @param  None
 * @retval None
 */
void LCD_DMA_Init(void)
{
    memset(&lcd_dma_handle, 0, sizeof(LCD_DMA_Handle_t));
    lcd_dma_handle.state = LCD_DMA_IDLE;
    lcd_dma_handle.callback = NULL;
    dma_transfer_complete = 1;
}

/**
 * @brief  获取DMA传输状态
 * @param  None
 * @retval LCD_DMA_State_t 当前DMA状态
 */
LCD_DMA_State_t LCD_DMA_GetState(void)
{
    return lcd_dma_handle.state;
}

/**
 * @brief  使用DMA填充单色矩形区域
 * @param  xsta: 起始X坐标
 * @param  ysta: 起始Y坐标
 * @param  xend: 结束X坐标
 * @param  yend: 结束Y坐标
 * @param  color: 填充颜色
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef LCD_DMA_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    if(lcd_dma_handle.state == LCD_DMA_BUSY)
    {
        return HAL_BUSY;
    }
    
    uint16_t width = xend - xsta;
    uint16_t height = yend - ysta;
    uint32_t total_pixels = width * height;
    
    // 设置显示窗口
    LCD_Address_Set(xsta, ysta, xend-1, yend-1);
    
    // 准备颜色数据
    uint16_t buffer_size = (total_pixels > LCD_DMA_MAX_BUFFER_SIZE/2) ? LCD_DMA_MAX_BUFFER_SIZE/2 : total_pixels;
    
    for(uint16_t i = 0; i < buffer_size; i++)
    {
        dma_color_buffer[i] = color;
    }
    
    lcd_dma_handle.state = LCD_DMA_BUSY;
    lcd_dma_handle.transfer_size = total_pixels;
    
    // 如果数据量小于缓冲区，一次性传输
    if(total_pixels <= LCD_DMA_MAX_BUFFER_SIZE/2)
    {
        dma_transfer_complete = 0;
        return HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)dma_color_buffer, total_pixels * 2);
    }
    else
    {
        // 分批传输
        uint32_t remaining = total_pixels;
        uint32_t transmitted = 0;
        
        while(remaining > 0)
        {
            uint32_t chunk_size = (remaining > LCD_DMA_MAX_BUFFER_SIZE/2) ? LCD_DMA_MAX_BUFFER_SIZE/2 : remaining;
            
            dma_transfer_complete = 0;
            HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)dma_color_buffer, chunk_size * 2);
            
            if(status != HAL_OK)
            {
                lcd_dma_handle.state = LCD_DMA_ERROR;
                return status;
            }
            
            // 等待传输完成
            while(!dma_transfer_complete);
            
            transmitted += chunk_size;
            remaining -= chunk_size;
        }
        
        lcd_dma_handle.state = LCD_DMA_COMPLETE;
        return HAL_OK;
    }
}

/**
 * @brief  使用DMA填充指定颜色数据的区域
 * @param  x: 起始X坐标
 * @param  y: 起始Y坐标
 * @param  width: 宽度
 * @param  height: 高度
 * @param  color_data: 颜色数据数组
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef LCD_DMA_FillArea(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *color_data)
{
    if(lcd_dma_handle.state == LCD_DMA_BUSY)
    {
        return HAL_BUSY;
    }
    
    uint32_t total_pixels = width * height;
    
    // 设置显示窗口
    LCD_Address_Set(x, y, x + width - 1, y + height - 1);
    
    lcd_dma_handle.state = LCD_DMA_BUSY;
    lcd_dma_handle.transfer_size = total_pixels;
    
    // 检查数据大小
    const uint32_t max_dma_transfer = 65535;
    
    if(total_pixels * 2 <= max_dma_transfer)
    {
        // 一次性传输
        dma_transfer_complete = 0;
        return HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)color_data, total_pixels * 2);
    }
    else
    {
        // 分批传输
        uint32_t remaining = total_pixels;
        uint32_t offset = 0;
        
        while(remaining > 0)
        {
            uint32_t chunk_size = (remaining * 2 > max_dma_transfer) ? max_dma_transfer/2 : remaining;
            
            dma_transfer_complete = 0;
            HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)(color_data + offset), chunk_size * 2);
            
            if(status != HAL_OK)
            {
                lcd_dma_handle.state = LCD_DMA_ERROR;
                return status;
            }
            
            // 等待传输完成
            while(!dma_transfer_complete);
            
            offset += chunk_size;
            remaining -= chunk_size;
        }
        
        lcd_dma_handle.state = LCD_DMA_COMPLETE;
        return HAL_OK;
    }
}

/**
 * @brief  LVGL专用DMA颜色填充（同步版本）
 * @param  sx: 起始X坐标
 * @param  sy: 起始Y坐标
 * @param  ex: 结束X坐标
 * @param  ey: 结束Y坐标
 * @param  color: LVGL颜色数组
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef LCD_DMA_LVGL_Fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, lv_color_t *color)
{
    if(lcd_dma_handle.state == LCD_DMA_BUSY)
    {
        return HAL_BUSY;
    }
    
    uint16_t width = ex - sx + 1;
    uint16_t height = ey - sy + 1;
    uint32_t total_pixels = width * height;
    
    // 设置显示窗口
    LCD_Address_Set(sx, sy, ex, ey);
    
    lcd_dma_handle.state = LCD_DMA_BUSY;
    
    // 转换LVGL颜色格式到LCD格式（如果需要）
    // 假设LVGL和LCD使用相同的RGB565格式
    
    const uint32_t max_dma_transfer = 65535;
    
    if(total_pixels * sizeof(lv_color_t) <= max_dma_transfer)
    {
        // 一次性传输
        dma_transfer_complete = 0;
        HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)color, total_pixels * sizeof(lv_color_t));
        
        if(status == HAL_OK)
        {
            // 等待传输完成
            while(!dma_transfer_complete);
            lcd_dma_handle.state = LCD_DMA_COMPLETE;
        }
        else
        {
            lcd_dma_handle.state = LCD_DMA_ERROR;
        }
        
        return status;
    }
    else
    {
        // 分批传输
        uint32_t remaining = total_pixels;
        uint32_t offset = 0;
        
        while(remaining > 0)
        {
            uint32_t chunk_size = (remaining * sizeof(lv_color_t) > max_dma_transfer) ? max_dma_transfer/sizeof(lv_color_t) : remaining;
            
            dma_transfer_complete = 0;
            HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)(color + offset), chunk_size * sizeof(lv_color_t));
            
            if(status != HAL_OK)
            {
                lcd_dma_handle.state = LCD_DMA_ERROR;
                return status;
            }
            
            // 等待传输完成
            while(!dma_transfer_complete);
            
            offset += chunk_size;
            remaining -= chunk_size;
        }
        
        lcd_dma_handle.state = LCD_DMA_COMPLETE;
        return HAL_OK;
    }
}

/**
 * @brief  LVGL专用DMA颜色填充（异步版本）
 * @param  sx: 起始X坐标
 * @param  sy: 起始Y坐标
 * @param  ex: 结束X坐标
 * @param  ey: 结束Y坐标
 * @param  color: LVGL颜色数组
 * @param  callback: 传输完成回调函数
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef LCD_DMA_LVGL_Fill_Async(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, lv_color_t *color, LCD_DMA_Callback_t callback)
{
    if(lcd_dma_handle.state == LCD_DMA_BUSY)
    {
        return HAL_BUSY;
    }
    
    uint16_t width = ex - sx + 1;
    uint16_t height = ey - sy + 1;
    uint32_t total_pixels = width * height;
    
    // 设置显示窗口
    LCD_Address_Set(sx, sy, ex, ey);
    
    lcd_dma_handle.state = LCD_DMA_BUSY;
    lcd_dma_handle.callback = callback;
    lcd_dma_handle.data_ptr = (uint8_t*)color;
    lcd_dma_handle.transfer_size = total_pixels;
    
    // 启动异步传输
    dma_transfer_complete = 0;
    return HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)color, total_pixels * sizeof(lv_color_t));
}

/**
 * @brief  使用DMA绘制位图
 * @param  x: 起始X坐标
 * @param  y: 起始Y坐标
 * @param  width: 位图宽度
 * @param  height: 位图高度
 * @param  bitmap: 位图数据
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef LCD_DMA_DrawBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *bitmap)
{
    return LCD_DMA_FillArea(x, y, width, height, (uint16_t*)bitmap);
}

/**
 * @brief  设置DMA传输完成回调函数
 * @param  callback: 回调函数指针
 * @retval None
 */
void LCD_DMA_SetCallback(LCD_DMA_Callback_t callback)
{
    lcd_dma_handle.callback = callback;
}

/**
 * @brief  等待DMA传输完成
 * @param  timeout_ms: 超时时间（毫秒）
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef LCD_DMA_WaitForComplete(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    
    while(lcd_dma_handle.state == LCD_DMA_BUSY)
    {
        if(HAL_GetTick() - start_tick > timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }
    
    if(lcd_dma_handle.state == LCD_DMA_ERROR)
    {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief  SPI DMA传输完成回调函数
 * @param  hspi: SPI句柄
 * @retval None
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI2)
    {
        dma_transfer_complete = 1;
        lcd_dma_handle.state = LCD_DMA_COMPLETE;
        
        // 调用用户回调函数
        if(lcd_dma_handle.callback != NULL)
        {
            lcd_dma_handle.callback();
            lcd_dma_handle.callback = NULL;
        }
    }
}

/**
 * @brief  SPI DMA传输错误回调函数
 * @param  hspi: SPI句柄
 * @retval None
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI2)
    {
        dma_transfer_complete = 1;
        lcd_dma_handle.state = LCD_DMA_ERROR;
        
        // 错误处理
        if(lcd_dma_handle.callback != NULL)
        {
            lcd_dma_handle.callback();
            lcd_dma_handle.callback = NULL;
        }
    }
}