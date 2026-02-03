#ifndef UART_TASK_H
#define UART_TASK_H

float calculate_step_size(uint8_t data_value);
void ProcessDataFrame(uint8_t* data ,uint8_t len);
void UARTTask_Entry(void * argument);


#endif

