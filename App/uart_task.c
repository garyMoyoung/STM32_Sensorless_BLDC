#include "cmsis_os.h"
#include "math.h"
#include "main.h"
#include "uart_task.h"
#include "pid.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern volatile float Mech_Angle;
extern float Elec_Angle;
extern volatile float Mech_RPM;
extern Iabc_Struct Iabc_M0;
extern Iqd_Struct Iqd_M0;
extern float pitch, roll, yaw;
extern SVPWM_Struct SVPWM_M0;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t rx1_buffer[BUFFER_SIZE];
extern osMessageQueueId_t IMUQueueHandle;
extern osMessageQueueId_t FOCQueueHandle;
extern osMessageQueueId_t PIDQueueHandle;
extern osMessageQueueId_t UARTQueueHandle;
extern FrameRxHandler frameHandler_one;
extern uint8_t rx1_frame_buffer[BUFFER_SIZE];
extern volatile uint16_t rx1_frame_len;
extern volatile uint8_t recv1_end_flag;
extern volatile uint32_t TIM9_ISR_CNT;
extern volatile uint32_t TIM10_ISR_CNT;

extern PIDController PID_Current_D;
extern PIDController PID_Current_Q;
extern PIDController PID_Speed;
extern PIDController PID_Position;

extern volatile FOC_Mode_t g_foc_mode;
extern uint16_t Diag_RawAdc[3];
extern float Diag_RawVolt[3];
extern uint8_t Diag_CurrentFault;
extern uint16_t current_adc_offset[3];
extern float OpenLoop_AlignUd_V;
extern float OpenLoop_RunUq_V;
extern float OpenLoop_TargetElecHz;
extern uint8_t FOC_RequestRecalibration(void);
extern void FOC_SetMode(uint8_t mode);

extern volatile uint8_t g_lcd_enable;
extern void LCD_SetEnable(uint8_t enable);

static PID_Param_t Id_temp;
static PID_Param_t Iq_temp;
static PID_Param_t Speed_temp;
static PID_Param_t Position_temp;
static uint8_t pid_temp_initialized = 0;
UART_Frame_t drame_task;

#define PC_CMD_READ_TELEMETRY   0x80U
#define PC_CMD_STREAM_ON        0x81U
#define PC_CMD_STREAM_OFF       0x82U
#define PC_CMD_READ_PID         0x83U
#define PC_CMD_READ_ALL         0x84U
#define PC_CMD_READ_DEBUG       0x85U
#define PC_CMD_READ_RAW_ADC     0x86U
#define PC_CMD_RECALIBRATE      0x87U
#define PC_CMD_SET_MODE         0x90U
#define PC_CMD_DISARM           0x91U
#define PC_CMD_SET_LCD_ENABLE   0x93U

#define OPEN_LOOP_DEBUG_ENABLE  1U

static volatile uint8_t telemetry_stream_enabled = 0;
static volatile uint16_t telemetry_stream_period_ms = 50;
static uint16_t telemetry_stream_cnt = 0;
static uint16_t uart2_speed_debug_cnt = 0;

static uint8_t uart2_adc_tx_buf[48];
static volatile uint8_t uart2_adc_tx_busy = 0U;

void UART1_SendByte(uint8_t ch)
{
    (void)HAL_UART_Transmit(&huart1, &ch, 1U, 50U);
}

void UART1_SendBytes(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 500U);
}

float calculate_step_size(uint8_t data_value)
{
    float step_size = 1.0f;
    uint8_t i;

    for(i = 0; i < data_value; i++)
    {
        step_size /= 10.0f;
    }

    return step_size;
}

static void UART_PrintTelemetry(void)
{
    printf("$TEL,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
           "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
           "%u,%u,%u,%u,%.4f,%.4f,%.4f,%u,%u#\r\n",
           Iabc_M0.Ia, Iabc_M0.Ib, Iabc_M0.Ic,
           Iqd_M0.Id, Iqd_M0.Iq,
           Mech_RPM, Mech_Angle, Elec_Angle,
           PID_Current_D.kp, PID_Current_D.ki, PID_Current_D.kd,
           PID_Current_Q.kp, PID_Current_Q.ki, PID_Current_Q.kd,
           PID_Speed.kp, PID_Speed.ki, PID_Speed.kd,
           PID_Position.kp, PID_Position.ki, PID_Position.kd,
           PID_Current_Q.target, PID_Speed.target,
           (unsigned int)g_foc_mode,
           (unsigned int)Diag_RawAdc[0], (unsigned int)Diag_RawAdc[1], (unsigned int)Diag_RawAdc[2],
           Diag_RawVolt[0], Diag_RawVolt[1], Diag_RawVolt[2],
           (unsigned int)Diag_CurrentFault,
           (unsigned int)g_lcd_enable);
}

static void UART_PrintRaw(void)
{
    printf("$RAW,%u,%.4f,%u,%u,%.4f,%u,%u,%.4f,%u,%u#\r\n",
           (unsigned int)Diag_RawAdc[0], Diag_RawVolt[0], (unsigned int)current_adc_offset[0],
           (unsigned int)Diag_RawAdc[1], Diag_RawVolt[1], (unsigned int)current_adc_offset[1],
           (unsigned int)Diag_RawAdc[2], Diag_RawVolt[2], (unsigned int)current_adc_offset[2],
           (unsigned int)Diag_CurrentFault);
}

static void UART_PrintDebug(void)
{
    printf("$DBG,%lu,%lu#\r\n", (unsigned long)TIM9_ISR_CNT, (unsigned long)TIM10_ISR_CNT);
}
static void UART_PrintPid(void)
{
    printf("$PID,ID,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f#\r\n",
           PID_Current_D.kp, PID_Current_D.ki, PID_Current_D.kd,
           PID_Current_D.target, PID_Current_D.output, PID_Current_D.integral);
    printf("$PID,IQ,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f#\r\n",
           PID_Current_Q.kp, PID_Current_Q.ki, PID_Current_Q.kd,
           PID_Current_Q.target, PID_Current_Q.output, PID_Current_Q.integral);
    printf("$PID,SPD,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f#\r\n",
           PID_Speed.kp, PID_Speed.ki, PID_Speed.kd,
           PID_Speed.target, PID_Speed.output, PID_Speed.integral);
    printf("$PID,POS,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f#\r\n",
           PID_Position.kp, PID_Position.ki, PID_Position.kd,
           PID_Position.target, PID_Position.output, PID_Position.integral);
}

static PIDController* UART_GetPidByName(const char *name)
{
    if (strcmp(name, "ID") == 0)
    {
        return &PID_Current_D;
    }
    if (strcmp(name, "IQ") == 0)
    {
        return &PID_Current_Q;
    }
    if (strcmp(name, "SPD") == 0)
    {
        return &PID_Speed;
    }
    if (strcmp(name, "POS") == 0)
    {
        return &PID_Position;
    }
    return NULL;
}

static void UART_CopyPidToParam(PID_Param_t *param, const PIDController *pid)
{
    param->kp = pid->kp;
    param->ki = pid->ki;
    param->kd = pid->kd;
    param->target = pid->target;
}

static void UART_InitPidTempFromLive(void)
{
    if (pid_temp_initialized != 0)
    {
        return;
    }

    UART_CopyPidToParam(&Id_temp, &PID_Current_D);
    UART_CopyPidToParam(&Iq_temp, &PID_Current_Q);
    UART_CopyPidToParam(&Speed_temp, &PID_Speed);
    UART_CopyPidToParam(&Position_temp, &PID_Position);
    pid_temp_initialized = 1;
}

static void UART_ApplyPidParam(PIDController *pid, const PID_Param_t *param)
{
    PID_param_set(pid, param->kp, param->ki, param->kd);
    pid->target = param->target;
}

static void UART_ProcessAsciiCommand(char *line)
{
    char *token;
    char *loop;
    char *target_str;
    char *kp_str;
    char *ki_str;
    char *kd_str;
    float target;
    float kp;
    float ki;
    float kd;
    char *end = strchr(line, '#');

    if (end != NULL)
    {
        *end = '\0';
    }

    token = strtok(line, ",");
    loop = strtok(NULL, ",");
    target_str = strtok(NULL, ",");
    kp_str = strtok(NULL, ",");
    ki_str = strtok(NULL, ",");
    kd_str = strtok(NULL, ",");

    if ((token != NULL) && (strcmp(token, "$WPID") == 0) &&
        (loop != NULL) && (target_str != NULL) &&
        (kp_str != NULL) && (ki_str != NULL) && (kd_str != NULL))
    {
        PIDController *pid = UART_GetPidByName(loop);
        if (pid == NULL)
        {
            printf("$ERR,PID,UNKNOWN_LOOP#\r\n");
            return;
        }

        target = (float)atof(target_str);
        kp = (float)atof(kp_str);
        ki = (float)atof(ki_str);
        kd = (float)atof(kd_str);

        PID_param_set(pid, kp, ki, kd);
        pid->target = target;
        pid_temp_initialized = 0;
        printf("$ACK,PID,%s,%.5f,%.5f,%.5f,%.5f#\r\n", loop, target, kp, ki, kd);
        UART_PrintPid();
        return;
    }

    if ((token != NULL) && (strcmp(token, "$MODE") == 0) && (loop != NULL))
    {
        FOC_SetMode((uint8_t)atoi(loop));
        printf("$ACK,MODE,%u#\r\n", (unsigned int)g_foc_mode);
        return;
    }

    if ((token != NULL) && (strcmp(token, "$LCD") == 0) && (loop != NULL))
    {
        LCD_SetEnable((uint8_t)atoi(loop));
        printf("$ACK,LCD,%u#\r\n", (unsigned int)g_lcd_enable);
        return;
    }

    printf("$ERR,UNKNOWN_CMD#\r\n");
}

void UART_TelemetryTick(void)
{
    if (telemetry_stream_enabled == 0)
    {
        telemetry_stream_cnt = 0;
        return;
    }

    if (++telemetry_stream_cnt >= telemetry_stream_period_ms)
    {
        telemetry_stream_cnt = 0;
        UART_PrintTelemetry();
    }
}

static void UART2_PrintSpeedDebug(void)
{
    char line[96];
    int len;

    len = snprintf(line, sizeof(line), "$SPD,%.4f,%.4f,%.4f#\r\n",
                   Mech_RPM, Mech_Angle, Elec_Angle);
    if ((len > 0) && (len < (int)sizeof(line)))
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 20);
    }
}

void UART2_SpeedDebugTick(void)
{
    if (++uart2_speed_debug_cnt >= 100)
    {
        uart2_speed_debug_cnt = 0;
        UART2_PrintSpeedDebug();
    }
}

void UART2_SendRawAdc(uint32_t tick, const uint16_t raw_adc[3])
{
#if (OPEN_LOOP_DEBUG_ENABLE != 0U)
    int len;

    if ((uart2_adc_tx_busy != 0U) || (huart2.gState != HAL_UART_STATE_READY))
    {
        return;
    }

    len = snprintf((char *)uart2_adc_tx_buf, sizeof(uart2_adc_tx_buf),
                   "$ADC,%lu,%u,%u,%u\r\n",
                   (unsigned long)tick,
                   (unsigned int)raw_adc[0],
                   (unsigned int)raw_adc[1],
                   (unsigned int)raw_adc[2]);

    if ((len <= 0) || ((uint32_t)len >= sizeof(uart2_adc_tx_buf)))
    {
        return;
    }

    uart2_adc_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&huart2, uart2_adc_tx_buf, (uint16_t)len) != HAL_OK)
    {
        uart2_adc_tx_busy = 0U;
    }
#else
    (void)tick;
    (void)raw_adc;
#endif
}

void UART2_RawAdcTxComplete(void)
{
    uart2_adc_tx_busy = 0U;
}

void ProcessDataFrame(uint8_t* data, uint8_t Proc_flag)
{
    uint8_t data2;
    PIDController *pid_to_apply = NULL;
    PID_Param_t *param_to_apply = NULL;
    float step_size;

    if(Proc_flag != 1)
    {
        return;
    }

    data2 = data[1];
    step_size = calculate_step_size(data[2]);
    UART_InitPidTempFromLive();

    switch(data[0])
    {
      case PC_CMD_READ_TELEMETRY:
        UART_PrintTelemetry();
      break;

      case PC_CMD_STREAM_ON:
        telemetry_stream_period_ms = data2;
        if (telemetry_stream_period_ms == 0)
        {
          telemetry_stream_period_ms = 50;
        }
        telemetry_stream_enabled = 1;
        printf("$ACK,STREAM_ON,%u#\r\n", telemetry_stream_period_ms);
      break;

      case PC_CMD_STREAM_OFF:
        telemetry_stream_enabled = 0;
        printf("$ACK,STREAM_OFF#\r\n");
      break;

      case PC_CMD_READ_PID:
        UART_PrintPid();
      break;

      case PC_CMD_READ_ALL:
        UART_PrintTelemetry();
        UART_PrintPid();
        UART_PrintDebug();
      break;

      case PC_CMD_READ_DEBUG:
        UART_PrintDebug();
      break;

      case PC_CMD_READ_RAW_ADC:
        UART_PrintRaw();
      break;

      case PC_CMD_RECALIBRATE:
        if (FOC_RequestRecalibration() != 0U)
        {
          printf("$ACK,CAL#\r\n");
          UART_PrintRaw();
        }
        else
        {
          printf("$ERR,CAL,BUSY#\r\n");
        }
      break;

      case PC_CMD_SET_MODE:
        FOC_SetMode(data2);
        printf("$ACK,MODE,%u#\r\n", (unsigned int)g_foc_mode);
      break;

      case PC_CMD_DISARM:
        FOC_SetMode((uint8_t)FOC_MODE_IDLE);
        printf("$ACK,MODE,%u#\r\n", (unsigned int)g_foc_mode);
      break;

      case PC_CMD_SET_LCD_ENABLE:
        LCD_SetEnable(data2);
        printf("$ACK,LCD,%u#\r\n", (unsigned int)g_lcd_enable);
      break;

      case 0x00:
      break;

      case 0x01:
        if(data2 == 0x01)       Id_temp.kp += step_size;
        else if(data2 == 0x11)  Id_temp.kp -= step_size;
        else if(data2 == 0x02)  Id_temp.ki += step_size;
        else if(data2 == 0x12)  Id_temp.ki -= step_size;
        else if(data2 == 0x03)  Id_temp.kd += step_size;
        else if(data2 == 0x13)  Id_temp.kd -= step_size;
        else if(data2 == 0x04)  Id_temp.target += step_size;
        else if(data2 == 0x14)  Id_temp.target -= step_size;
        pid_to_apply = &PID_Current_D;
        param_to_apply = &Id_temp;
      break;

      case 0x02:
        if(data2 == 0x01)       Iq_temp.kp += step_size;
        else if(data2 == 0x11)  Iq_temp.kp -= step_size;
        else if(data2 == 0x02)  Iq_temp.ki += step_size;
        else if(data2 == 0x12)  Iq_temp.ki -= step_size;
        else if(data2 == 0x03)  Iq_temp.kd += step_size;
        else if(data2 == 0x13)  Iq_temp.kd -= step_size;
        else if(data2 == 0x04)  Iq_temp.target += step_size;
        else if(data2 == 0x14)  Iq_temp.target -= step_size;
        pid_to_apply = &PID_Current_Q;
        param_to_apply = &Iq_temp;
      break;

      case 0x03:
        if(data2 == 0x01)       Iq_temp.target += 2.0f;
        else if(data2 == 0x02)  Iq_temp.target -= 2.0f;
        pid_to_apply = &PID_Current_Q;
        param_to_apply = &Iq_temp;
      break;

      case 0x04:
        if(data2 == 0x01)       Speed_temp.kp += step_size;
        else if(data2 == 0x11)  Speed_temp.kp -= step_size;
        else if(data2 == 0x02)  Speed_temp.ki += step_size;
        else if(data2 == 0x12)  Speed_temp.ki -= step_size;
        else if(data2 == 0x03)  Speed_temp.kd += step_size;
        else if(data2 == 0x13)  Speed_temp.kd -= step_size;
        else if(data2 == 0x04)  Speed_temp.target += step_size;
        else if(data2 == 0x14)  Speed_temp.target -= step_size;
        pid_to_apply = &PID_Speed;
        param_to_apply = &Speed_temp;
      break;

      case 0x05:
        if(data2 == 0x01)       Position_temp.kp += step_size;
        else if(data2 == 0x11)  Position_temp.kp -= step_size;
        else if(data2 == 0x02)  Position_temp.ki += step_size;
        else if(data2 == 0x12)  Position_temp.ki -= step_size;
        else if(data2 == 0x03)  Position_temp.kd += step_size;
        else if(data2 == 0x13)  Position_temp.kd -= step_size;
        else if(data2 == 0x04)  Position_temp.target += step_size;
        else if(data2 == 0x14)  Position_temp.target -= step_size;
        pid_to_apply = &PID_Position;
        param_to_apply = &Position_temp;
      break;

      case 0x06:
        if(data2 == 0x01)       OpenLoop_AlignUd_V += step_size;
        else if(data2 == 0x11)  OpenLoop_AlignUd_V -= step_size;
        printf("$ACK,OL_UD,%.4f#\r\n", OpenLoop_AlignUd_V);
      break;

      case 0x07:
        if(data2 == 0x01)       OpenLoop_RunUq_V += step_size;
        else if(data2 == 0x11)  OpenLoop_RunUq_V -= step_size;
        printf("$ACK,OL_UQ,%.4f#\r\n", OpenLoop_RunUq_V);
      break;

      case 0x08:
        if(data2 == 0x01)       OpenLoop_TargetElecHz += step_size;
        else if(data2 == 0x11)  OpenLoop_TargetElecHz -= step_size;
        printf("$ACK,OL_HZ,%.4f#\r\n", OpenLoop_TargetElecHz);
      break;

      default:
      break;
    }

    if ((pid_to_apply != NULL) && (param_to_apply != NULL))
    {
      UART_ApplyPidParam(pid_to_apply, param_to_apply);
      UART_PrintPid();
    }
}
void UART_ProcessInTimer(void)
{
  UART_Frame_t frame;
  uint16_t frame_len;
  uint16_t i;

  if (recv1_end_flag == 0)
  {
    return;
  }

  frame_len = rx1_frame_len;
  recv1_end_flag = 0;
  rx1_frame_len = 0;

  if ((frame_len > 0) && (rx1_frame_buffer[0] == '$'))
  {
    char ascii_cmd[BUFFER_SIZE + 1];
    uint16_t copy_len = frame_len;
    if (copy_len > BUFFER_SIZE)
    {
      copy_len = BUFFER_SIZE;
    }
    memcpy(ascii_cmd, rx1_frame_buffer, copy_len);
    ascii_cmd[copy_len] = '\0';
    UART_ProcessAsciiCommand(ascii_cmd);
    memset(rx1_frame_buffer, 0, frame_len);
    return;
  }

  for(i = 0; i < frame_len; i++)
  {
    switch(frameHandler_one.state)
    {
      case WAIT_HEAD1:
        if(rx1_frame_buffer[i] == 0xFE)
        {
          frameHandler_one.rxBuff[DOWN_FRAME_HEAD1_POS] = rx1_frame_buffer[i];
          frameHandler_one.state = WAIT_HEAD2;
        }
        break;

      case WAIT_HEAD2:
        if(rx1_frame_buffer[i] == 0xEF)
        {
          frameHandler_one.rxBuff[DOWN_FRAME_HEAD2_POS] = rx1_frame_buffer[i];
          frameHandler_one.state = WAIT_DEVICE;
        }
        break;

      case WAIT_DEVICE:
        frameHandler_one.rxBuff[DOWN_FRAME_DEVICE_POS] = rx1_frame_buffer[i];
        frameHandler_one.device = rx1_frame_buffer[i];
        frameHandler_one.state = WAIT_data1;
        break;

      case WAIT_data1:
        /* ARG0 是完整的数据字节(0~255),不是长度字段,不应按 DOWN_FRAME_LEN_MAX 过滤,
         * 否则 ARG0>30 的命令(例如 0x81 STREAM_ON 默认周期50ms)会卡在此状态永远收不到帧尾 */
        frameHandler_one.rxBuff[DOWN_FRAME_DATA_POS] = rx1_frame_buffer[i];
        frameHandler_one.data[0] = rx1_frame_buffer[i];
        frameHandler_one.state = WAIT_data2;
        break;

      case WAIT_data2:
        frameHandler_one.rxBuff[DOWN_FRAME_DATA_POS + 1] = rx1_frame_buffer[i];
        frameHandler_one.data[1] = rx1_frame_buffer[i];
        frameHandler_one.state = WAIT_TAIL1;
        break;

      case WAIT_TAIL1:
        if(rx1_frame_buffer[i] == 0x23)
        {
          frameHandler_one.rxBuff[DOWN_FRAME_TAIL1_POS] = rx1_frame_buffer[i];
          frameHandler_one.state = WAIT_TAIL2;
        }
        break;

      case WAIT_TAIL2:
        if(rx1_frame_buffer[i] == 0x24)
        {
          frameHandler_one.rxBuff[DOWN_FRAME_TAIL2_POS] = rx1_frame_buffer[i];
          frameHandler_one.frameOK = true;
          frame.flag = 1;
          frameHandler_one.state = WAIT_HEAD1;
        }
        break;

      default:
        frameHandler_one.state = WAIT_HEAD1;
        break;
    }

    if(frameHandler_one.frameOK == true)
    {
      frame.data[0] = frameHandler_one.device;
      memcpy(&frame.data[1], frameHandler_one.data, 2);
      ProcessDataFrame(frame.data, frame.flag);
      frameHandler_one.frameOK = false;
    }
  }

  memset(rx1_frame_buffer, 0, frame_len);
}

void UARTTask_Entry(void * argument)
{
  UART_InitPidTempFromLive();

  for(;;)
  {
    UART_ProcessInTimer();
    UART_TelemetryTick();
    UART2_SpeedDebugTick();

    osStatus_t status_uart = osMessageQueueGet(UARTQueueHandle, &drame_task, NULL, 0);
    if (status_uart == osOK)
    {
        ProcessDataFrame(drame_task.data, drame_task.flag);
    }

    osDelay(1);
  }
}
