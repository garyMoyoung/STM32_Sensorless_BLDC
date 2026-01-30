#include "cmsis_os.h"
#include "math.h"
#include "main.h"

extern float Mech_Angle; 
extern Iabc_Struct Iabc_M0;
extern float pitch, roll, yaw;
extern SVPWM_Struct SVPWM_M0;
extern UART_HandleTypeDef huart1;
extern osMessageQueueId_t IMUQueueHandle;
extern osMessageQueueId_t FOCQueueHandle;
void UARTTask_Entry(void * argument)
{
  /* USER CODE BEGIN UARTTask_Entry */
  static uint8_t dma_buffer[256];
  int len;
  
  IMU_Euler_t euler_data;
  for(;;)
  {
    // 每 2ms 执行一次（500Hz）
    // 
    // osStatus_t status = osMessageQueueGet(IMUQueueHandle, &euler_data, NULL, 0);  // 非阻塞获取
    // if (status == osOK) {
    //     pitch = euler_data.pitch;
    //     roll = euler_data.roll;
    //     yaw = euler_data.yaw;
    // }
    FOC_Data_t foc_data;
    osStatus_t status = osMessageQueueGet(FOCQueueHandle, &foc_data, NULL, 0);
    len = sprintf((char *)dma_buffer, 
        "ta:tb:tc:Ang:ia:ib:ic:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        foc_data.tcm1, foc_data.tcm2, foc_data.tcm3, Mech_Angle,
        foc_data.Ia, foc_data.Ib, foc_data.Ic);
    HAL_UART_Transmit_DMA(&huart1, dma_buffer, len);
    printf("uart_task_working\n");
    osDelay(10);  // 500Hz
  }
  /* USER CODE END UARTTask_Entry */
}

