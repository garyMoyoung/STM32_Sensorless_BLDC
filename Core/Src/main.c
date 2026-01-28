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
#include "lvgl.h"                // 它为整个LVGL提供了更完整的头文件引用
#include "lv_port_disp.h"        // LVGL的显示支持
#include "lv_port_indev.h"       // LVGL的触屏支持
#include "init_file.h"
#include "foc_drv.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
float Udc = 12.6f;
uint16_t ADC_Value[3] = {0};
uint32_t ADC_buff[3] = {0};
uint16_t ad_val_orig[3] = {0};
AS5600_T AS5600;
float Mech_Angle = 0.0f; 
float Elec_Angle = 0.0f;   
float Mech_RPM = 0.0f;

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

extern osMutexId imuDataMutexHandle;
extern float pitch_inside, roll_inside, yaw_inside;
extern uint32_t imu_task_counter;  // 引入IMU任务计数器
uint32_t TIM9_100Hz_CNT = 0;  // 100Hz定时器计数器
uint32_t TIM10_task_CNT = 0;
float angle = 0.0f;
PIDController PID_Current_D;
PIDController PID_Current_Q;
Key_Struct_init Key[3];
// PID Q 参数备用变量（用于按键调整）
float PID_Q_Kp = 0.0f;
float PID_Q_Ki = 0.0f;
float PID_Q_Kd = 0.0f;

/* KEY_init BEGIN*/
// 更灵敏的阈值（单位：ms），适配1kHz采样
const uint16_t DEBOUNCE_MS = 5;       // 5ms 消抖
const uint16_t SHORT_MS = 200;        // 短按判定 200ms
const uint16_t LONG_MS = 500;         // 长按判定 500ms（更灵敏）
const uint16_t DOUBLE_MS = 300;       // 双击最大间隔 300ms
const uint16_t SINGLE_MS = 150;       // 单击确认阈值 150ms
/* KEY_init END*/

/* UASRT BEGIN*/
FrameRxHandler frameHandler_one;
uint8_t rx1_buffer[100]={0};
volatile uint8_t rx1_len = 0;
volatile uint8_t recv1_end_flag = 0;
volatile uint8_t rx1_addr = 0;
volatile uint8_t rx1_length = 0;
volatile uint8_t rx1_data[8] = {0};
volatile uint8_t rx1_checksum = 0;
volatile uint8_t rx1_checksum_flag = 0;
/* UASRT END*/

/* CodeSet BEGIN*/
T_FlipFlop_t Key_Y;
uint8_t Key_clk = 0;
uint8_t T_input1 = 0;
Edge_Detector_t edge_detector;

/* CodeSet END*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* Private function prototypes -----------------------------------------------*/
/* ------------------通过重定向将printf函数映射到串口1上-------------------*/

#if !defined(__MICROLIB)

//#pragma import(__use_no_semihosting)
__asm (".global __use_no_semihosting\n\t");
void _sys_exit(int x) //避免使用半主机模式
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
  /* 实现串口发送一个字节数据的函数 */
  //serial_write(&serial1, (uint8_t)ch); //发送一个自己的数据到串口
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
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
  __HAL_TIM_CLEAR_IT(&htim3,TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim3);
  __HAL_TIM_CLEAR_IT(&htim9,TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim9);
  __HAL_TIM_CLEAR_IT(&htim10,TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim10);
  
  HAL_ADCEx_InjectedStart(&hadc1);
  __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOC);
  //HAL_ADC_Start_DMA(&hadc3,(uint32_t *)ADC_buff,15);
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_buff,12);
  HAL_ADC_PollForConversion(&hadc1, 50);
  HAL_ADC_Start(&hadc1);

  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1680);
  svpwm_init(&Udq_M0,0.0f,2.5f);
  AS5600_Init(&AS5600,&hi2c2);
  PID_Init(&PID_Current_D,3.0f,-3.0f,100.0f);
  PID_Init(&PID_Current_Q,3.0f,-3.0f,100.0f);
  PID_param_set(&PID_Current_D,0.0f,0.0f,0.0f);
  PID_param_set(&PID_Current_Q,0.0f,0.0f,0.0f);
  // 初始化 PID Q 参数本地副本
  PID_Q_Kp = 0.0f;
  PID_Q_Ki = 0.0f;
  PID_Q_Kd = 0.0f;
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc->Instance == ADC1)
  {
    ad_val_orig[0] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
    ad_val_orig[1] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
    ad_val_orig[2] = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
    Iabc_M0.Ia = ((ad_val_orig[0]*3.3f)/4096.0f -1.65f)*4.0f;
    Iabc_M0.Ib = ((ad_val_orig[1]*3.3f)/4096.0f -1.65f)*4.0f;
    Iabc_M0.Ic = ((ad_val_orig[2]*3.3f)/4096.0f -1.65f)*4.0f;


  }

}
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
      // AS5600_Update(&AS5600);
      // Mech_Angle = AS5600_GetOnceAngle(&AS5600);
      // Elec_Angle = Mech_Angle*7.0f;
      // Mech_RPM = AS5600_GetVelocity_RPM(&AS5600);
      // Clarke_transform(&Iabc_M0,&Ialpbe_M0);
      // Park_transform(&Iqd_M0,&Ialpbe_M0,Elec_Angle);

      angle += 0.04f;
      if(angle > 6.2831853f) angle = 0.0f;
      inverseParkTransform(&Udq_M0,&Ualpbe_M0,angle*7);
      svpwm_sector_choice(&SVPWM_M0,&Ualpbe_M0);
      SVPWM_timer_period_set(&SVPWM_M0,&Ualpbe_M0);
      // PWM_TIM2_Set(3360*SVPWM_M0.ta,3360*SVPWM_M0.tb,3360*SVPWM_M0.tc);

      // 处理按键调整 PID_Current_Q.Kp/Ki/Kd

      // printf("pitch:roll:yaw:ia:ib:ic:Ang:Rpm:Counter:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%lu\n", 
      //                 pitch,roll,yaw,Iabc_M0.Ia,Iabc_M0.Ib,Iabc_M0.Ic,
      //                 Mech_Angle,Mech_RPM,imu_task_counter);
      // printf("Ang:Rpm:ia:ib:ic:id:iq:kp:Uq:Ud:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n"
      // ,Mech_Angle,Mech_RPM,Iabc_M0.Ia,Iabc_M0.Ib,Iabc_M0.Ic
      // ,Iqd_M0.Id,Iqd_M0.Iq,PID_Current_Q.kp,Udq_M0.Uq,Udq_M0.Ud);
      printf("ta:tb:tc:%.4f,%.4f,%.4f\n"
        ,SVPWM_M0.tcm1,SVPWM_M0.tcm2,SVPWM_M0.tcm3);
    
  }
  if (htim->Instance == TIM10) // 1ms tick
  {
    Key_read();  // 只调用一次
    if(++TIM10_task_CNT >= 1000)
    {
      TIM10_task_CNT = 0;
      
    }
    if (Key[0].mode_now == 1) {
        PID_Q_Kp += 0.001f;
      }
      if (Key[1].mode_now == 1) {
        PID_Q_Kp -= 0.001f;
      }
    PID_Current_Q.kp = PID_Q_Kp;
    PID_Current_D.kp = PID_Q_Kp;
    // 将按键计数视为毫秒（每次中断 +1 ms）
    for (uint8_t i = 0; i < 3; i++)
    {
        if (Key[i].Time_Count_Flag == 1)
        {
            Key[i].Press_Time_Count++; // 单位：ms
        }
        else
        {
            Key[i].Press_Time_Count = 0;
        }

        

        // 如果按键持续按下且超过长按阈值，立即判定为长按（无需等待释放）
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
