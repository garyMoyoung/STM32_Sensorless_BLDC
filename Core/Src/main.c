/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "stdio.h"
#include "stdarg.h"
#include "AS5600.h"
#include "string.h"
#include "lcd_init.h"
#include "lcd.h"
#include "stdarg.h"
#include <string.h>
#include "fifo.h"
#include "MPU9250-DMP.h"
#include "MPU9250_RegisterMap.h"
#include "math.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "stm32_mpu9250_clk.h"
#include "stm32_mpu9250_i2c.h"
#include "Algorithmic.h"
#include "lvgl.h"                // ??????LVGL??????????????????
#include "lv_port_disp.h"        // LVGL????????
#include "lv_port_indev.h"       // LVGL????????
#include "init_file.h"
#include "foc_drv.h"
#include "pid.h"
#include "timer_utils.h"
#include "uart_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CURRENT_ADC_REF_VOLTAGE      3.3f
#define CURRENT_ADC_FULL_SCALE       4096.0f
#define CURRENT_INA240_GAIN          50.0f
#define CURRENT_SHUNT_RESISTOR_OHM   0.005f
#define CURRENT_ADC_TO_AMP           (CURRENT_ADC_REF_VOLTAGE / CURRENT_ADC_FULL_SCALE / (CURRENT_INA240_GAIN * CURRENT_SHUNT_RESISTOR_OHM))
#define CURRENT_OFFSET_DEFAULT_ADC   2048U
/* 标定出的零电流偏置若偏离 CURRENT_OFFSET_DEFAULT_ADC 超过此阈值(约0.48V),视为该相模拟前端异常 */
#define CURRENT_OFFSET_FAULT_TOL_ADC 600U

#define OPEN_LOOP_DEBUG_ENABLE       1U
#define OPEN_LOOP_ALIGN_TIME_MS      500U
#define OPEN_LOOP_RAMP_TIME_MS       3000U
#define OPEN_LOOP_ALIGN_UD_V_DEFAULT     1.0f
#define OPEN_LOOP_RUN_UQ_V_DEFAULT       1.2f
#define OPEN_LOOP_TARGET_ELEC_HZ_DEFAULT 5.0f
#define OPEN_LOOP_POLE_PAIRS         7.0f
#define OPEN_LOOP_UART_PERIOD_MS     50U
#define OPEN_LOOP_PWM_PERIOD         3360.0f
#define OPEN_LOOP_TWO_PI             6.283185307f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// float Init_Iabc[3] , Iabc[3];//predict
Udq_Struct Udq_M0;
Ualpbe_Struct Ualpbe_M0;
Iabc_Struct Iabc_M0;
Ialpbe_Struct Ialpbe_M0;
Ialpbe_Struct Ialpbe_M0_last;
Iqd_Struct Iqd_M0;
SVPWM_Struct SVPWM_M0;
AS5600 M0;
float Udc = 12.6f;
uint16_t ADC_Value[3] = {0};
uint32_t ADC_buff[3] = {0};
uint16_t ad_val_orig[3] = {0};
volatile uint16_t uart2_adc_raw[3] = {0U, 0U, 0U};
volatile uint32_t uart2_adc_raw_tick = 0U;
static uint8_t uart2_adc_tx_buf[48];
static volatile uint8_t uart2_adc_tx_busy = 0U;
uint16_t current_adc_offset[3] = {
  CURRENT_OFFSET_DEFAULT_ADC,
  CURRENT_OFFSET_DEFAULT_ADC,
  CURRENT_OFFSET_DEFAULT_ADC
};

/* 电流采样诊断量: 原始ADC计数/换算电压/零点异常标志(bit0=A,bit1=B,bit2=C), 供上位机 $RAW / $TEL 读取 */
uint16_t Diag_RawAdc[3] = {0U, 0U, 0U};
float Diag_RawVolt[3] = {0.0f, 0.0f, 0.0f};
uint8_t Diag_CurrentFault = 0U;

/* FOC 运行模式(运行时可通过UART切换), 上电默认IDLE(PWM零输出), 避免上电即自动转动 */
volatile FOC_Mode_t g_foc_mode = FOC_MODE_IDLE;

/* 开环调试参数, 默认值取自 OPEN_LOOP_*_DEFAULT, 可通过UART在线调节(见 ProcessDataFrame case 0x06~0x08) */
float OpenLoop_AlignUd_V = OPEN_LOOP_ALIGN_UD_V_DEFAULT;
float OpenLoop_RunUq_V = OPEN_LOOP_RUN_UQ_V_DEFAULT;
float OpenLoop_TargetElecHz = OPEN_LOOP_TARGET_ELEC_HZ_DEFAULT;

volatile float Mech_Angle = 0.0f;
float Elec_Angle = 0.0f;
volatile float Mech_RPM = 0.0f;
float Mech_RPM_filter = 0.0f;

uint16_t ucAdc[3];
uint8_t Flag;
float Udq[2] = {0 , 10};
float Uab[2];
float Iab[2] , Iab_Last[2];
float Idq[2];
double sincos[2];
uint32_t CNT;
uint16_t Counter;

float I_Target[2] = {0 , 0.55};
float C_Kp=0.7535 , C_Ki=0.000475;
float Speed , Speed_sum , Speed_New;
float Iq_sum;
uint16_t Speed_Target = 1000;
float S_Kp = 0.0125, S_Ki = 0.00028 , S_Kd = 0.021;
const float fc = 5000;
const float Ts = 0.00005;
float b = 2*3.1415926f*fc*Ts;
float a=0.1556;//b/(b+1)
uint8_t SpeedLoopCounter;
float We;
float R = 0.6 , Ld = 0.000558 , Lq = 0.000558 , T = 0.0007255 , flux = 0.005753;
float Iab_fore_New[2] , Iab_fore_Last[2];
float h = 2.5 , Vab[2] , Vab_Filter[2];
float Eab[2];

float Vab_alpha = 0.286;
float Theta_fore_New , Theta_fore_Last , We_fore;
float PLL_Kp =0.034 , PLL_Ki= 2.12;

float Alpha;

uint16_t CCNNTT;
uint16_t Start_Flag = 1, Start_CNT = 0;

extern float pitch_inside, roll_inside, yaw_inside;
extern uint32_t imu_task_counter;  // ???IMU????????
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
volatile uint32_t TIM9_ISR_CNT = 0;
volatile uint32_t TIM10_ISR_CNT = 0;
uint32_t TIM9_100Hz_CNT = 0;  // 100Hz?????????
uint32_t TIM10_task_CNT = 0;
float angle = 0.0f;
PIDController PID_Current_D;
PIDController PID_Current_Q;
PIDController PID_Speed;
PIDController PID_Position;
PID_Param_t Id_pid;
PID_Param_t Iq_pid;
PID_Param_t Speed_pid;
Key_Struct_init Key[3];
// PID Q ?????????????????????
float PID_Q_Kp = 0.0f;
float PID_Q_Ki = 0.0f;
float PID_Q_Kd = 0.0f;

/* KEY_init BEGIN*/
// ????????????????s??????1kHz???
const uint16_t DEBOUNCE_MS = 5;       // 5ms ???
const uint16_t SHORT_MS = 200;        // ?????? 200ms
const uint16_t LONG_MS = 500;         // ?????? 500ms????????
const uint16_t DOUBLE_MS = 300;       // ??????????300ms
const uint16_t SINGLE_MS = 150;       // ??????????150ms
/* KEY_init END*/

/* UASRT BEGIN*/
FrameRxHandler frameHandler_one;
uint8_t rx1_buffer[BUFFER_SIZE] = {0};
volatile uint16_t rx1_len = 0;
volatile uint8_t recv1_end_flag = 0;
volatile uint8_t rx1_addr = 0;
volatile uint16_t rx1_length = 0;
volatile uint8_t rx1_data[8] = {0};
volatile uint8_t rx1_checksum = 0;
volatile uint8_t rx1_checksum_flag = 0;
uint8_t rx1_frame_buffer[BUFFER_SIZE] = {0};
volatile uint16_t rx1_frame_len = 0;

/* UASRT END*/

/* CodeSet BEGIN*/
T_FlipFlop_t Key_Y;
uint8_t Key_clk = 0;
uint8_t T_input1 = 0;
Edge_Detector_t edge_detector;
/* CodeSet END*/

/* IMU BEGIN*/
extern osMessageQueueId_t IMUQueueHandle;
extern osMessageQueueId_t FOCQueueHandle;
extern osMessageQueueId_t PIDQueueHandle;
extern osMessageQueueId_t UARTQueueHandle;
/* IMU END*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void Current_ReadRaw(uint16_t raw_adc[3]);
static void Current_CalibrateOffset(uint16_t sample_count);
static void OpenLoop_Control(void);
static void Uart2_SendRawAdc(uint32_t tick, const uint16_t raw_adc[3]);

/* Private function prototypes -----------------------------------------------*/
/* ------------------?????????printf?????????????------------------*/

#if !defined(__MICROLIB)

//#pragma import(__use_no_semihosting)
__asm (".global __use_no_semihosting\n\t");
void _sys_exit(int x) //??????????????
{
  x = x;
}
//__use_no_semihosting was requested, but _ttywrch was 
void _ttywrch(int ch)
{
    ch = ch;
}
struct __FILE
{
  int handle;
};
FILE __stdout;

#endif

#if defined ( __GNUC__ ) && !defined (__clang__) 
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
PUTCHAR_PROTOTYPE
{
  UART1_SendByte((uint8_t)ch);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  // Init_All();
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//??????
  // __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);//??????
  HAL_UART_Receive_DMA(&huart1,rx1_buffer,BUFFER_SIZE);

  __HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim3);
  __HAL_TIM_CLEAR_IT(&htim9,TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim9);
  
  HAL_ADCEx_InjectedStart(&hadc1);
  __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOC);
  
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_buff,12);
  HAL_ADC_PollForConversion(&hadc1, 50);
  HAL_ADC_Start(&hadc1);
  // HAL_ADCEx_InjectedStart_IT(&hadc1);

  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1680);
  Current_CalibrateOffset(512U);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);

  svpwm_init(&Udq_M0,0.0f,0.0f);

  AS5600_Init(&M0,&hi2c2);
  DWT_Init();
  PID_Init(&PID_Current_D,4.0f,-4.0f,100.0f);
  PID_Init(&PID_Current_Q,4.0f,-4.0f,100.0f);
  PID_Init(&PID_Speed,15.0f,-15.0f,0.0f);
  PID_Init(&PID_Position,15.0f,-15.0f,0.0f);
  PID_param_set(&PID_Current_D,0.0517f,0.0f,0.0f);
  PID_param_set(&PID_Current_Q,0.0517f,0.0f,0.0f);
  PID_param_set(&PID_Speed,0.0f,0.0f,0.0f);
  PID_param_set(&PID_Position,0.0f,0.0f,0.0f);
  PID_Current_D.target = 0.0f;
  PID_Current_Q.target = 0.0f;
  PID_Speed.target = 1000.0f;
  PID_Position.target = 0.0f;
  // ?????PID Q ?????????
  __HAL_TIM_CLEAR_IT(&htim10,TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim10);
  

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void PWM_TIM2_Set(uint16_t pwm_a,uint16_t pwm_b,uint16_t pwm_c)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_a);//
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_b);//
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_c);//
}

static uint16_t PWM_LimitCompare(float compare)
{
    if (compare < 0.0f)
    {
        return 0U;
    }
    if (compare > OPEN_LOOP_PWM_PERIOD)
    {
        return (uint16_t)OPEN_LOOP_PWM_PERIOD;
    }
    return (uint16_t)compare;
}

/*
 * 注入通道->物理引脚映射(adc.c: rank1=CH2/PA2, rank2=CH3/PA3, rank3=CH4/PA4):
 *   raw_adc[0] (Ia) <- InjectedRank3 <- ADC1_IN4 <- PA4
 *   raw_adc[1] (Ib) <- InjectedRank2 <- ADC1_IN3 <- PA3
 *   raw_adc[2] (Ic) <- InjectedRank1 <- ADC1_IN2 <- PA2
 * 这是当前代码假设的相序对应关系,未与原理图/实物核实过。拿到板子后可用
 * $RAW 诊断帧(逐引脚给出原始计数与电压)配合万用表/示波器逐相验证,
 * 如与实际接线不符,只需调整这里的下标映射,不影响其余算法。
 */
static void Current_ReadRaw(uint16_t raw_adc[3])
{
    raw_adc[2] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
    raw_adc[1] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
    raw_adc[0] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
}

static void Uart2_SendRawAdc(uint32_t tick, const uint16_t raw_adc[3])
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

static void Current_CalibrateOffset(uint16_t sample_count)
{
    uint32_t sum[3] = {0U, 0U, 0U};
    uint16_t raw_adc[3] = {0U, 0U, 0U};
    uint16_t valid_samples = 0U;

    if (sample_count == 0U)
    {
        sample_count = 1U;
    }

    for (uint16_t i = 0U; i < sample_count; i++)
    {
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 2U) == HAL_OK)
        {
            Current_ReadRaw(raw_adc);
            sum[0] += raw_adc[0];
            sum[1] += raw_adc[1];
            sum[2] += raw_adc[2];
            valid_samples++;
        }
    }

    if (valid_samples > 0U)
    {
        uint16_t new_offset[3];
        uint8_t ch;

        new_offset[0] = (uint16_t)(sum[0] / valid_samples);
        new_offset[1] = (uint16_t)(sum[1] / valid_samples);
        new_offset[2] = (uint16_t)(sum[2] / valid_samples);

        Diag_CurrentFault = 0U;
        for (ch = 0U; ch < 3U; ch++)
        {
            int32_t dev = (int32_t)new_offset[ch] - (int32_t)CURRENT_OFFSET_DEFAULT_ADC;
            if ((dev > (int32_t)CURRENT_OFFSET_FAULT_TOL_ADC) || (dev < -(int32_t)CURRENT_OFFSET_FAULT_TOL_ADC))
            {
                /* 标定值偏离过大,判定该相模拟前端异常,零点仍用默认值,不采信可疑标定结果 */
                Diag_CurrentFault |= (uint8_t)(1U << ch);
            }
            else
            {
                current_adc_offset[ch] = new_offset[ch];
            }
        }
    }
}

void Current_read()
{
    uint8_t ch;

    Current_ReadRaw(ad_val_orig);
    Iabc_M0.Ia = ((float)ad_val_orig[0] - (float)current_adc_offset[0]) * CURRENT_ADC_TO_AMP;
    Iabc_M0.Ib = ((float)ad_val_orig[1] - (float)current_adc_offset[1]) * CURRENT_ADC_TO_AMP;
    Iabc_M0.Ic = ((float)ad_val_orig[2] - (float)current_adc_offset[2]) * CURRENT_ADC_TO_AMP;

    for (ch = 0U; ch < 3U; ch++)
    {
        Diag_RawAdc[ch] = ad_val_orig[ch];
        Diag_RawVolt[ch] = (float)ad_val_orig[ch] * (CURRENT_ADC_REF_VOLTAGE / CURRENT_ADC_FULL_SCALE);
    }
}

/**
 * @brief 供UART命令(0x87 RECALIBRATE)调用的重新标零入口
 * @retval 1=已执行标定, 0=当前不在IDLE模式,拒绝执行(避免在电机运行中把工作点当零点)
 */
uint8_t FOC_RequestRecalibration(void)
{
    if (g_foc_mode != FOC_MODE_IDLE)
    {
        return 0U;
    }
    Current_CalibrateOffset(512U);
    return 1U;
}

/**
 * @brief 供UART命令(0x90 SET_MODE / 0x91 DISARM)调用的模式切换入口
 */
void FOC_SetMode(uint8_t mode)
{
    if (mode > (uint8_t)FOC_MODE_POSITION)
    {
        return;
    }
    g_foc_mode = (FOC_Mode_t)mode;
}



void Queue_proc()
{
    // osMessageQueueGet(PIDQueueHandle, &Id_pid, NULL, 0);
    // PID_param_set(&PID_Current_D,0.050f,Id_pid.ki,Id_pid.kd);
    // osMessageQueueGet(PIDQueueHandle, &Iq_pid, NULL, 0);
    // PID_param_set(&PID_Current_Q,0.050f,Iq_pid.ki,Iq_pid.kd);
    osMessageQueueGet(PIDQueueHandle, &Speed_pid, NULL, 0);
    PID_param_set(&PID_Speed,Speed_pid.kp,Speed_pid.ki,Speed_pid.kd);
    // PID_Current_Q.target = Iq_pid.target;
}

#define FOC_LOOP_DT_S  0.001f

static float prev_speed_angle = 0.0f;
static uint8_t prev_speed_valid = 0U;

static void MechSpeed_Update(float current_angle)
{
    float angle_delta;

    if (prev_speed_valid == 0U)
    {
        prev_speed_angle = current_angle;
        prev_speed_valid = 1U;
        return;
    }

    angle_delta = current_angle - prev_speed_angle;
    if (angle_delta > 3.14159f)
    {
        angle_delta -= 2.0f * 3.14159f;
    }
    else if (angle_delta < -3.14159f)
    {
        angle_delta += 2.0f * 3.14159f;
    }

    Mech_RPM = rad_sec_to_rpm(angle_delta / FOC_LOOP_DT_S);
    prev_speed_angle = current_angle;
}

void angle_proc()
{
    float raw_angle;

    AS5600_UpdateAngle(&M0);
    raw_angle = AS5600_GetAngle(&M0);
    MechSpeed_Update(raw_angle);
    Mech_Angle = _normalizeAngle(raw_angle);
    Elec_Angle = 7.0f * Mech_Angle;
}


/* 电流环: 只跑 D/Q 电流PID,不参与速度/位置级联。Id/Iq target 由上位机通过既有
 * PID写命令(二进制0x01/0x02步进 或 ASCII $WPID,ID/IQ,...#)直接设置。 */
static void FOC_CurrentLoop_Step(void)
{
    angle_proc();
    Current_read();
    Clarke_transform(&Iabc_M0,&Ialpbe_M0);
    Park_transform(&Iqd_M0,&Ialpbe_M0,Elec_Angle);

    Udq_M0.Ud = PID_Position_Calculate(&PID_Current_D,PID_Current_D.target,Iqd_M0.Id);
    Udq_M0.Uq = PID_Position_Calculate(&PID_Current_Q,PID_Current_Q.target,Iqd_M0.Iq);
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    PWM_TIM2_Set((uint16_t)(3360*SVPWM_M0.tcm1),(uint16_t)(3360*SVPWM_M0.tcm2),(uint16_t)(3360*SVPWM_M0.tcm3));
}

/* 速度环: 速度PID输出作为Iq目标,再走电流环。SPD target 由上位机设置(PID_Speed.target)。 */
static void FOC_SpeedLoop_Step(void)
{
    angle_proc();
    Current_read();
    Clarke_transform(&Iabc_M0,&Ialpbe_M0);
    Park_transform(&Iqd_M0,&Ialpbe_M0,Elec_Angle);

    PID_Current_Q.target = -PID_Position_Calculate(&PID_Speed,PID_Speed.target,Mech_RPM);

    Udq_M0.Ud = PID_Position_Calculate(&PID_Current_D,PID_Current_D.target,Iqd_M0.Id);
    Udq_M0.Uq = PID_Position_Calculate(&PID_Current_Q,PID_Current_Q.target,Iqd_M0.Iq);
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    PWM_TIM2_Set((uint16_t)(3360*SVPWM_M0.tcm1),(uint16_t)(3360*SVPWM_M0.tcm2),(uint16_t)(3360*SVPWM_M0.tcm3));
}

/* 位置环: 位置PID(角度误差按最短路径归一化)输出作为速度目标,再走速度环->电流环。
 * POS target 由上位机设置(PID_Position.target,单位: 机械角 rad,范围不限,内部按最短路径归一化)。 */
static void FOC_PositionLoop_Step(void)
{
    float wrapped_err;

    angle_proc();
    Current_read();
    Clarke_transform(&Iabc_M0,&Ialpbe_M0);
    Park_transform(&Iqd_M0,&Ialpbe_M0,Elec_Angle);

    wrapped_err = AngleErrorWrap(PID_Position.target - Mech_Angle);
    PID_Speed.target = PID_Position_Calculate(&PID_Position, Mech_Angle + wrapped_err, Mech_Angle);

    PID_Current_Q.target = -PID_Position_Calculate(&PID_Speed,PID_Speed.target,Mech_RPM);

    Udq_M0.Ud = PID_Position_Calculate(&PID_Current_D,PID_Current_D.target,Iqd_M0.Id);
    Udq_M0.Uq = PID_Position_Calculate(&PID_Current_Q,PID_Current_Q.target,Iqd_M0.Iq);
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    PWM_TIM2_Set((uint16_t)(3360*SVPWM_M0.tcm1),(uint16_t)(3360*SVPWM_M0.tcm2),(uint16_t)(3360*SVPWM_M0.tcm3));
}

static void OpenLoop_Control(void)
{
    static uint32_t open_loop_tick = 0U;
    static float open_loop_elec_angle = 0.0f;
    uint16_t raw_adc[3] = {0U, 0U, 0U};
    float ramp = 1.0f;
    float elec_hz = 0.0f;

    Current_ReadRaw(raw_adc);
    ad_val_orig[0] = raw_adc[0];
    ad_val_orig[1] = raw_adc[1];
    ad_val_orig[2] = raw_adc[2];
    uart2_adc_raw[0] = raw_adc[0];
    uart2_adc_raw[1] = raw_adc[1];
    uart2_adc_raw[2] = raw_adc[2];
    uart2_adc_raw_tick = open_loop_tick;

    if ((open_loop_tick % OPEN_LOOP_UART_PERIOD_MS) == 0U)
    {
        Uart2_SendRawAdc(open_loop_tick, raw_adc);
    }

    if (open_loop_tick < OPEN_LOOP_ALIGN_TIME_MS)
    {
        open_loop_elec_angle = 0.0f;
        Udq_M0.Ud = OpenLoop_AlignUd_V;
        Udq_M0.Uq = 0.0f;
        Mech_RPM = 0.0f;
    }
    else
    {
        uint32_t run_tick = open_loop_tick - OPEN_LOOP_ALIGN_TIME_MS;

        if (run_tick < OPEN_LOOP_RAMP_TIME_MS)
        {
            ramp = (float)run_tick / (float)OPEN_LOOP_RAMP_TIME_MS;
        }

        elec_hz = OpenLoop_TargetElecHz * ramp;
        Udq_M0.Ud = 0.0f;
        Udq_M0.Uq = OpenLoop_RunUq_V * ramp;
        open_loop_elec_angle += OPEN_LOOP_TWO_PI * elec_hz * FOC_LOOP_DT_S;
        open_loop_elec_angle = _normalizeAngle(open_loop_elec_angle);
        Mech_RPM = (elec_hz / OPEN_LOOP_POLE_PAIRS) * 60.0f;
    }

    Elec_Angle = open_loop_elec_angle;
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    PWM_TIM2_Set(PWM_LimitCompare(OPEN_LOOP_PWM_PERIOD * SVPWM_M0.tcm1),
                 PWM_LimitCompare(OPEN_LOOP_PWM_PERIOD * SVPWM_M0.tcm2),
                 PWM_LimitCompare(OPEN_LOOP_PWM_PERIOD * SVPWM_M0.tcm3));

    open_loop_tick++;

    if (open_loop_tick == 0U)
    {
        /* 溢出后从对齐阶段重新开始,避免长时间运行后 tick 溢出导致状态错乱 */
        open_loop_tick = 1U;
    }
}

/* PWM 归零(50%对称占空,三相电压差为0),用于IDLE及模式切换瞬间,避免残留占空比造成冲击 */
static void FOC_OutputZero(void)
{
    PWM_TIM2_Set((uint16_t)(OPEN_LOOP_PWM_PERIOD * 0.5f),
                 (uint16_t)(OPEN_LOOP_PWM_PERIOD * 0.5f),
                 (uint16_t)(OPEN_LOOP_PWM_PERIOD * 0.5f));
}

/* 1kHz(TIM10)调用入口: 按 g_foc_mode 分发到对应的调试模式,模式切换瞬间清空四个PID的
 * 积分/误差历史并把PWM归零,避免上一模式残留的积分/目标突变造成电流或转速冲击。 */
void FOC_ModeDispatch(void)
{
    static FOC_Mode_t prev_mode = FOC_MODE_IDLE;

    if (g_foc_mode != prev_mode)
    {
        PID_Reset(&PID_Current_D);
        PID_Reset(&PID_Current_Q);
        PID_Reset(&PID_Speed);
        PID_Reset(&PID_Position);
        FOC_OutputZero();
        prev_mode = g_foc_mode;
    }

    switch (g_foc_mode)
    {
        case FOC_MODE_OPEN_LOOP:
            OpenLoop_Control();
            break;
        case FOC_MODE_CURRENT:
            FOC_CurrentLoop_Step();
            break;
        case FOC_MODE_SPEED:
            FOC_SpeedLoop_Step();
            break;
        case FOC_MODE_POSITION:
            FOC_PositionLoop_Step();
            break;
        case FOC_MODE_IDLE:
        default:
            /* 空闲态仍刷新角度/电流用于监控,但不输出非零电压 */
            angle_proc();
            Current_read();
            FOC_OutputZero();
            break;
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc->Instance == ADC1)
  {
      Current_read();
  }

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
      uart2_adc_tx_busy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
      uart2_adc_tx_busy = 0U;
  }
}
//7ee75574453dedc37120c1ddd948099600afb96d
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM9)
  {
    TIM9_ISR_CNT++;
  }
  if (htim->Instance == TIM10) // 1ms tick
  {
    TIM10_ISR_CNT++;

    FOC_ModeDispatch();
    Key_read();
    if(++TIM10_task_CNT >= 1000)
    {
      TIM10_task_CNT = 0;
      
    }

    // ????????????????????? +1 ms??
    for (uint8_t i = 0; i < 3; i++)
    {
        if (Key[i].Time_Count_Flag == 1)
        {
            Key[i].Press_Time_Count++; // ?????s
        }
        else
        {
            Key[i].Press_Time_Count = 0;
        }

        

        // ???????????????????????????????????????????????
        if (Key[i].Time_Count_Flag == 1 && Key[i].Press_Time_Count >= LONG_MS && Key[i].Key_Long_Flag == 0) {
          Key[i].Key_Long_Flag = 1;
          Key[i].mode = 2;
          Key[i].mode_now = 2;
          Key[i].Step = 0;
          Key[i].Time_Count_Flag = 0;
        }

        switch (Key[i].Step)
        {
            case 0:
            {
                if (Key[i].Key_State == 0) {
                    Key[i].Step = 1;
                    Key[i].Time_Count_Flag = 1;
                } else {
                    Key[i].mode_now = 0;
                }
            } break;
            case 1:
            {
                if (Key[i].Press_Time_Count > DEBOUNCE_MS && Key[i].Key_State == 0) {
                    Key[i].Step = 2;
                }
            } break;
            case 2:
            {
                if (Key[i].Key_State == 1) {
                    if (Key[i].Press_Time_Count < SHORT_MS) {
                        Key[i].Step = 3;
                    }
                    if (Key[i].Press_Time_Count > LONG_MS) {
                        Key[i].Step = 0;
                        Key[i].Time_Count_Flag = 0;
                        Key[i].Key_Long_Flag = 1;
                        Key[i].mode = 2;
                        Key[i].mode_now = 2;
                    }
                }
            } break;
            case 3:
            {
                if (Key[i].Key_State == 0 && Key[i].Press_Time_Count < DOUBLE_MS) {
                    Key[i].Step = 4;
                    Key[i].Time_Count_Flag = 0;
                    Key[i].Key_Double_Flag = 1;
                    Key[i].mode = 3;
                    Key[i].mode_now = 3;
                } else if (Key[i].Key_State == 1 && Key[i].Press_Time_Count >= SINGLE_MS) {
                    Key[i].Step = 4;
                    Key[i].Time_Count_Flag = 0;
                    Key[i].Key_Single_Flag = 1;
                    Key[i].mode = 1;
                    Key[i].mode_now = 1;
                }
            } break;
            case 4:
            {
                if (Key[i].Key_State == 1) {
                    Key[i].Step = 0;
                }
            } break;
        }
    }
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
