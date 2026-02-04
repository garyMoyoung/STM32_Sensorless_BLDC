#include "cmsis_os.h"
#include "math.h"
#include "main.h"
#include "uart_task.h"

extern float Mech_Angle; 
extern Iabc_Struct Iabc_M0;
extern float pitch, roll, yaw;
extern SVPWM_Struct SVPWM_M0;
extern UART_HandleTypeDef huart1;
extern osMessageQueueId_t IMUQueueHandle;
extern osMessageQueueId_t FOCQueueHandle;
extern osMessageQueueId_t PIDQueueHandle;
extern osMessageQueueId_t UARTQueueHandle;
extern FrameRxHandler frameHandler_one;

extern PIDController PID_Current_D;
extern PIDController PID_Current_Q;
static PID_Param_t Id_temp;
static PID_Param_t Iq_temp;
static PID_Param_t Speed_temp;
UART_Frame_t drame_task;
float calculate_step_size(uint8_t data_value) {
    // 步长 = 10^(-data_value)
    float step_size = 1.0f;
    
    for(int i = 0; i < data_value; i++) {
        step_size /= 10.0f;
    }
    
    return step_size;
}

// 数据帧处理函数
void ProcessDataFrame(uint8_t* data, uint8_t Proc_flag) 
{
    // 这里处理接收到的4字节数据
    // 根据实际需求解析数据
    if(Proc_flag == 1) 
    {
        uint8_t data1 = data[0];//三环选取
        uint8_t data2 = data[1];//参数索引
        uint8_t data3 = data[2];//步长
        float step_size = calculate_step_size(data[2]);

        switch(data[0])
        {
          case 0x00:
          break;
          case 0x01://电流环参数
            if((data2 == 0x01)&&Proc_flag == 1)       Id_temp.kp += step_size;
            else if((data2 == 0x11)&&Proc_flag == 1)  Id_temp.kp -= step_size;
            else if((data2 == 0x02)&&Proc_flag == 1)  Id_temp.ki += step_size;
            else if((data2 == 0x12)&&Proc_flag == 1)  Id_temp.ki -= step_size;
          break;
          case 0x02://电流环参数
            if((data2 == 0x01)&&Proc_flag == 1)       Iq_temp.kp += step_size;
            else if((data2 == 0x11)&&Proc_flag == 1)  Iq_temp.kp -= step_size;
            else if((data2 == 0x02)&&Proc_flag == 1)  Iq_temp.ki += step_size;
            else if((data2 == 0x12)&&Proc_flag == 1)  Iq_temp.ki -= step_size;
          break;
          case 0x03:
            if((data2 == 0x01)&&Proc_flag == 1)       Iq_temp.target += 2.0f;
            else if((data2 == 0x02)&&Proc_flag == 1)  Iq_temp.target -= 2.0f;
          break;
          case 0x04://速度环参数
            if((data2 == 0x01)&&Proc_flag == 1)       Speed_temp.kp += step_size;
            else if((data2 == 0x11)&&Proc_flag == 1)  Speed_temp.kp -= step_size;
            else if((data2 == 0x02)&&Proc_flag == 1)  Speed_temp.ki += step_size;
            else if((data2 == 0x12)&&Proc_flag == 1)  Speed_temp.ki -= step_size;
          break;
          default:
          break;
        }
        Proc_flag = 0;
        osMessageQueuePut(PIDQueueHandle, &Id_temp, 0, 0);
        osMessageQueuePut(PIDQueueHandle, &Iq_temp, 0, 0);
        osMessageQueuePut(PIDQueueHandle, &Speed_temp, 0, 0);

        // printf("data1:0x%02x,data2:0x%02x,data3:0x%02x\n",data1,data2,data3);
        // printf("Id_temp.kp:%.4f,Id_temp.ki:%.4f,Iq_temp.kp:%.4f,Iq_temp.ki:%.4f\n",
        // Id_temp.kp,Id_temp.ki,Iq_temp.kp,Iq_temp.ki);
    }
    
}

void UARTTask_Entry(void * argument)
{
  Id_temp.kp = 0.0f;
  Id_temp.ki = 0.0f;
  Iq_temp.kp = 0.0f;
  Iq_temp.ki = 0.0f;
  /* USER CODE BEGIN UARTTask_Entry */
  for(;;)
  {
    osStatus_t status_uart = osMessageQueueGet(UARTQueueHandle, &drame_task, NULL, 0);
    if (status_uart == osOK) {
        ProcessDataFrame(drame_task.data,drame_task.flag);
    }
    // osStatus_t status = osMessageQueueGet(IMUQueueHandle, &euler_data, NULL, 0);  // 非阻塞获取
    // if (status == osOK) {
    //     pitch = euler_data.pitch;
    //     roll = euler_data.roll;
    //     yaw = euler_data.yaw;
    // }
    
    osDelay(1);  // 500Hz
  }
  /* USER CODE END UARTTask_Entry */
}

