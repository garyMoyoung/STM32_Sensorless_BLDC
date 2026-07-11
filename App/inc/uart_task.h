#ifndef UART_TASK_H
#define UART_TASK_H

float calculate_step_size(uint8_t data_value);
void ProcessDataFrame(uint8_t* data ,uint8_t len);
void UART_ProcessInTimer(void);
void UART_TelemetryTick(void);
void UART2_SpeedDebugTick(void);
void UART1_SendByte(uint8_t ch);
void UART1_SendBytes(const uint8_t *data, uint16_t len);


#endif

