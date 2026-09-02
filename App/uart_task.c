#include "cmsis_os.h"
#include "math.h"
#include "main.h"
#include "uart_task.h"
#include "pid.h"
#include "pid_storage.h"
#include "SMO.h"
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

/* SMO(滑模观测器)影子估算输出,见 Bsp/SMO.c SMO_ShadowUpdate()——只做估算,
   不参与实际PID闭环,这里单纯把估算值发给上位机和真实AS5600角度/转速对比。 */
extern float Theta_fore_New;
extern float We_fore;
extern float Eab[2];
extern float Vab_Filter[2];

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
extern uint8_t FOC_RequestElectricalAlignment(void);
extern uint8_t FOC_IsElectricalAlignmentValid(void);
extern uint8_t FOC_TakeElectricalAlignmentResult(float *offset_rad);
extern uint8_t FOC_TakeCurrentSaturationTrip(uint16_t raw_adc[3]);

extern volatile uint8_t g_lcd_enable;
extern void LCD_SetEnable(uint8_t enable);
extern volatile uint8_t g_lvgl_demo_enable;
extern void LvglDemo_SetEnable(uint8_t enable);

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
#define PC_CMD_ELECTRICAL_ALIGN 0x92U
#define PC_CMD_SET_MODE         0x90U
#define PC_CMD_DISARM           0x91U
#define PC_CMD_SET_LCD_ENABLE   0x93U
#define PC_CMD_SET_LVGL_ENABLE  0x94U
#define PC_CMD_SAVE_PID         0x88U
#define PC_CMD_LOAD_PID         0x89U

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
    /* SMO是"影子"估算(见FOC_*Loop_Step里的SMO_ShadowUpdate),不参与控制,
       这里只是把它估的角度/转速换算成和ElecAngle/RPM同单位,方便上位机直接
       对比两条曲线看收敛/跟踪效果。 */
    float smo_rpm = We_fore * 60.0f / (2.0f * 3.1415926f * MOTOR_POLE_PAIRS);

    printf("$TEL,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
           "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
           "%u,%u,%u,%u,%.4f,%.4f,%.4f,%u,%u,"
           "%.4f,%.4f,%.4f,%u,"
           "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f#\r\n",
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
           (unsigned int)g_lcd_enable,
           /* 第二台直流有刷电机(Bsp/dc_motor.c)当前工程里尚未实现/未接入,
              这里先占位输出0,只是为了让字段数量对上upmachine的TEL_FIELDS(42个),
              否则parse_tel()里的长度校验会直接返回None,导致波形区/读数栏永远
              收不到解析后的数据(哪怕原始$TEL,...文本已经正确收到)。等
              dc_motor.c真正接入后,把这4个0替换成真实的DcAngle/DcRPM/DcDuty/DcMode。 */
           0.0f, 0.0f, 0.0f, 0U,
           /* SMO调试字段(追加在末尾,upmachine两处TEL_FIELDS要同步加):
              SmoTheta=估计电角度(rad,0~2π) SmoWe=估计电角速度(rad/s)
              SmoRPM=换算成机械RPM,和RPM字段同单位可直接对比
              SmoEa/SmoEb=估计反电动势alpha/beta SmoVfA/SmoVfB=PLL输入(滑模面低通后) */
           Theta_fore_New, We_fore, smo_rpm,
           Eab[0], Eab[1], Vab_Filter[0], Vab_Filter[1]);
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

static void UART_PrintOnePid(const char *name, const PIDController *pid)
{
    printf("$PID,%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f#\r\n",
           name, pid->kp, pid->ki, pid->kd, pid->target, pid->output, pid->integral);
}

static void UART_PrintPid(void)
{
    UART_PrintOnePid("ID",  &PID_Current_D);
    UART_PrintOnePid("IQ",  &PID_Current_Q);
    UART_PrintOnePid("SPD", &PID_Speed);
    UART_PrintOnePid("POS", &PID_Position);
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

/* 保存/加载都只在IDLE模式下允许:擦除128KB扇区最坏耗时约2s,期间flash总线被整体占用,
   电机运行中调用会让FOC ISR失步,与0x87(重新标定)的限制同理 */
static void UART_SavePid(void)
{
    if (g_foc_mode != FOC_MODE_IDLE)
    {
        printf("$ERR,PIDSAVE,BUSY#\r\n");
        return;
    }

    if (PidStorage_Save() != 0U)
    {
        printf("$ACK,PIDSAVE#\r\n");
    }
    else
    {
        printf("$ERR,PIDSAVE,FAIL#\r\n");
    }
}

static void UART_LoadPid(void)
{
    if (g_foc_mode != FOC_MODE_IDLE)
    {
        printf("$ERR,PIDLOAD,BUSY#\r\n");
        return;
    }

    if (PidStorage_Load() != 0U)
    {
        pid_temp_initialized = 0;
        printf("$ACK,PIDLOAD#\r\n");
        UART_PrintPid();
    }
    else
    {
        printf("$ERR,PIDLOAD,EMPTY#\r\n");
    }
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
        if (((uint8_t)atoi(loop) >= (uint8_t)FOC_MODE_CURRENT) &&
            (FOC_IsElectricalAlignmentValid() == 0U))
        {
            printf("$ERR,MODE,NOT_ALIGNED#\r\n");
        }
        else
        {
            printf("$ACK,MODE,%u#\r\n", (unsigned int)g_foc_mode);
        }
        return;
    }

    if ((token != NULL) && (strcmp(token, "$ALIGN") == 0))
    {
        if (FOC_RequestElectricalAlignment() != 0U)
        {
            printf("$ACK,ALIGN,START#\r\n");
        }
        else
        {
            printf("$ERR,ALIGN,BUSY#\r\n");
        }
        return;
    }

    if ((token != NULL) && (strcmp(token, "$LCD") == 0) && (loop != NULL))
    {
        LCD_SetEnable((uint8_t)atoi(loop));
        printf("$ACK,LCD,%u#\r\n", (unsigned int)g_lcd_enable);
        return;
    }

    if ((token != NULL) && (strcmp(token, "$LVGL") == 0) && (loop != NULL))
    {
        LvglDemo_SetEnable((uint8_t)atoi(loop));
        printf("$ACK,LVGL,%u#\r\n", (unsigned int)g_lvgl_demo_enable);
        return;
    }

    if ((token != NULL) && (strcmp(token, "$SAVEPID") == 0))
    {
        UART_SavePid();
        return;
    }

    if ((token != NULL) && (strcmp(token, "$LOADPID") == 0))
    {
        UART_LoadPid();
        return;
    }

    printf("$ERR,UNKNOWN_CMD#\r\n");
}

void UART_TelemetryTick(void)
{
    float offset_rad;
    uint16_t saturation_raw[3];
    uint8_t align_result = FOC_TakeElectricalAlignmentResult(&offset_rad);

    if (align_result == 1U)
    {
        printf("$ACK,ALIGN,DONE,%.5f#\r\n", offset_rad);
    }
    else if (align_result == 2U)
    {
        printf("$ERR,ALIGN,ADC_SATURATED#\r\n");
    }

    if (FOC_TakeCurrentSaturationTrip(saturation_raw) != 0U)
    {
        printf("$TRIP,ADC_SATURATED,%u,%u,%u#\r\n",
               (unsigned int)saturation_raw[0],
               (unsigned int)saturation_raw[1],
               (unsigned int)saturation_raw[2]);
    }

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

/* field_code 编码: 0x01/0x11 = kp +/-, 0x02/0x12 = ki +/-, 0x03/0x13 = kd +/-, 0x04/0x14 = target +/- */
static void UART_AdjustPidParam(PID_Param_t *param, uint8_t field_code, float step)
{
    switch (field_code)
    {
      case 0x01: param->kp     += step; break;
      case 0x11: param->kp     -= step; break;
      case 0x02: param->ki     += step; break;
      case 0x12: param->ki     -= step; break;
      case 0x03: param->kd     += step; break;
      case 0x13: param->kd     -= step; break;
      case 0x04: param->target += step; break;
      case 0x14: param->target -= step; break;
      default: break;
    }
}

static void UART_AdjustFloatParam(float *value, uint8_t field_code, float step)
{
    if (field_code == 0x01)
    {
        *value += step;
    }
    else if (field_code == 0x11)
    {
        *value -= step;
    }
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

      case PC_CMD_ELECTRICAL_ALIGN:
        if (FOC_RequestElectricalAlignment() != 0U)
        {
          printf("$ACK,ALIGN,START#\r\n");
        }
        else
        {
          printf("$ERR,ALIGN,BUSY#\r\n");
        }
      break;

      case PC_CMD_SET_MODE:
        FOC_SetMode(data2);
        if ((data2 >= (uint8_t)FOC_MODE_CURRENT) && (FOC_IsElectricalAlignmentValid() == 0U))
        {
          printf("$ERR,MODE,NOT_ALIGNED#\r\n");
        }
        else
        {
          printf("$ACK,MODE,%u#\r\n", (unsigned int)g_foc_mode);
        }
      break;

      case PC_CMD_DISARM:
        FOC_SetMode((uint8_t)FOC_MODE_IDLE);
        printf("$ACK,MODE,%u#\r\n", (unsigned int)g_foc_mode);
      break;

      case PC_CMD_SET_LCD_ENABLE:
        LCD_SetEnable(data2);
        printf("$ACK,LCD,%u#\r\n", (unsigned int)g_lcd_enable);
      break;

      case PC_CMD_SET_LVGL_ENABLE:
        LvglDemo_SetEnable(data2);
        printf("$ACK,LVGL,%u#\r\n", (unsigned int)g_lvgl_demo_enable);
      break;

      case PC_CMD_SAVE_PID:
        UART_SavePid();
      break;

      case PC_CMD_LOAD_PID:
        UART_LoadPid();
      break;

      case 0x00:
      break;

      case 0x01:
        UART_AdjustPidParam(&Id_temp, data2, step_size);
        pid_to_apply = &PID_Current_D;
        param_to_apply = &Id_temp;
      break;

      case 0x02:
        UART_AdjustPidParam(&Iq_temp, data2, step_size);
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
        UART_AdjustPidParam(&Speed_temp, data2, step_size);
        pid_to_apply = &PID_Speed;
        param_to_apply = &Speed_temp;
      break;

      case 0x05:
        UART_AdjustPidParam(&Position_temp, data2, step_size);
        pid_to_apply = &PID_Position;
        param_to_apply = &Position_temp;
      break;

      case 0x06:
        UART_AdjustFloatParam(&OpenLoop_AlignUd_V, data2, step_size);
        printf("$ACK,OL_UD,%.4f#\r\n", OpenLoop_AlignUd_V);
      break;

      case 0x07:
        UART_AdjustFloatParam(&OpenLoop_RunUq_V, data2, step_size);
        printf("$ACK,OL_UQ,%.4f#\r\n", OpenLoop_RunUq_V);
      break;

      case 0x08:
        UART_AdjustFloatParam(&OpenLoop_TargetElecHz, data2, step_size);
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
        else if(rx1_frame_buffer[i] == 0xFE)
        {
          /* 允许 0xFE 0xFE 0xEF ... 这类帧头重复的情况下重新同步,而不是直接丢帧回到WAIT_HEAD1
           * 导致这个0xFE被跳过。 */
          frameHandler_one.state = WAIT_HEAD2;
        }
        else
        {
          frameHandler_one.state = WAIT_HEAD1;
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
        else
        {
          /* 帧尾1不匹配说明这一帧已经错位,必须重新同步到WAIT_HEAD1,
           * 否则状态机会永远卡在WAIT_TAIL1,后续所有正确帧都无法被识别,
           * 只能靠重新上电复位状态机才能恢复通信。 */
          frameHandler_one.state = WAIT_HEAD1;
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
        else
        {
          /* 同上,帧尾2不匹配也必须复位状态机,避免永久卡死。 */
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
