#ifndef UART_TASK_H
#define UART_TASK_H

float calculate_step_size(uint8_t data_value);
void ProcessDataFrame(uint8_t* data ,uint8_t len);
void UART_ProcessInTimer(void);
void UART_TelemetryTick(void);
void UART2_SpeedDebugTick(void);
void UART1_SendByte(uint8_t ch);
void UART1_SendBytes(const uint8_t *data, uint16_t len);
void UARTTask_Entry(void *argument);

/* 开环调试原始ADC通过UART2周期性发送(供不接上位机主协议时用示波器/串口助手直接观察) */
void UART2_SendRawAdc(uint32_t tick, const uint16_t raw_adc[3]);
/* USART2 DMA发送完成/出错回调(在main.c的HAL回调里调用,清除发送忙标志) */
void UART2_RawAdcTxComplete(void);


#endif

