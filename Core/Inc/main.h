/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define DOWN_FRAME_HEADER1    0x24   // 下行帧头1 (PC->MCU)
#define DOWN_FRAME_HEADER2    0x42   // 下行帧头2 (PC->MCU)
#define DOWN_FRAME_LEN_MIN    5       // 最小帧长度:双帧头+地址+长度+校验
#define DOWN_FRAME_LEN_MAX    30      // 最大帧长度
#define DOWN_FRAME_HEAD1_POS  0       // 第一帧头位置
#define DOWN_FRAME_HEAD2_POS  1       // 第二帧头位置
#define DOWN_FRAME_DEVICE_POS   2       // 设备位置 
#define DOWN_FRAME_DATA_POS    3       // 数据位置
#define DOWN_FRAME_TAIL1_POS    4       // 帧尾1位置
#define DOWN_FRAME_TAIL2_POS    5       // 帧尾2位置

typedef uint16_t u16;
typedef uint32_t u32;
typedef uint8_t u8;
typedef struct
{
    float Uq;
    float Ud;
}Udq_Struct;
typedef struct
{
    float U_alpha;
    float U_beta;
}Ualpbe_Struct;
typedef struct
{
    float Ia;
    float Ib;
    float Ic;
}Iabc_Struct;
typedef struct
{
    float I_alpha;
    float I_beta; 
}Ialpbe_Struct;
typedef struct
{
    float Id;
    float Iq;
}Iqd_Struct;
typedef struct
{
    int sector;
    float U1;
    float U2;
    float U3;  

    float Ux;
    float Uy;
    float Uz;

    float ta;
    float tb;
    float tc;

    float Ts;   
    float t1;
    float t2;
    float t0;
    float t7;
    float tcm1;
    float tcm2;
    float tcm3;
}SVPWM_Struct;

typedef struct {
    float kp;        // ????
    float ki;        // ???? 
    float kd;        // ????
    
    float error;     // ????
    float lastError; // ????
    float preError;  // ????
    
    float integral;  // ???????
    float output;    // PID???????
    
    float maxOutput; // ??????
    float minOutput; // ??????
    float maxIntegral; // ????
} PIDController;

typedef struct Key_Struct
{
	uint8_t Step;
	uint8_t Key_State;
	uint8_t Key_Single_Flag;//单击标志
	uint8_t Key_Long_Flag;//长按标志
	uint8_t Key_Double_Flag;//双击标志
	uint8_t Time_Count_Flag;//计时标志
	uint8_t Press_Time_Count;//按下时间计数
	uint8_t mode;//模式标志
	uint8_t mode_now;
}Key_Struct_init;

typedef enum {
    WAIT_HEAD1,     // 等待第一帧头
    WAIT_HEAD2,     // 等待第二帧头
    WAIT_DEVICE,      // 等待设备
    WAIT_data1,
    WAIT_data2,
    WAIT_data3,
    WAIT_TAIL1,
    WAIT_TAIL2
} RxState;
// 接收处理结构????
typedef struct {
    uint8_t  rxBuff[DOWN_FRAME_LEN_MAX];// 接收缓冲
    uint8_t  device;                      // 地址
    uint8_t  data[3];                     // 接收长度
    RxState  state;                     // 状态机状态
    bool     frameOK;                   // 帧完成标志
} FrameRxHandler;

typedef struct {
    float pitch;    // 俯仰角
    float roll;     // 滚转角
    float yaw;      // 偏航角
} IMU_Euler_t;

typedef struct {
    float tcm1;
    float tcm2;
    float tcm3;
    float Ia;
    float Ib;
    float Ic;
    float Iq;
    float Id;
    float Mech_Angle;
} FOC_Data_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float maxOutput;
    float minOutput;
    float maxIntegral;
} PID_Param_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TFT_RES_Pin GPIO_PIN_13
#define TFT_RES_GPIO_Port GPIOF
#define TFT_DC_Pin GPIO_PIN_14
#define TFT_DC_GPIO_Port GPIOF
#define TFT_CS_Pin GPIO_PIN_15
#define TFT_CS_GPIO_Port GPIOF
#define TFT_BL_Pin GPIO_PIN_0
#define TFT_BL_GPIO_Port GPIOG
#define Toggle_Pin GPIO_PIN_7
#define Toggle_GPIO_Port GPIOE
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOG
#define RGBA_Pin GPIO_PIN_8
#define RGBA_GPIO_Port GPIOB
#define RGB_B_Pin GPIO_PIN_9
#define RGB_B_GPIO_Port GPIOB
#define RGB_C_Pin GPIO_PIN_0
#define RGB_C_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define BUFFER_SIZE     256
#define RGBA_toggle() HAL_GPIO_TogglePin(RGBA_GPIO_Port, RGBA_Pin)
#define RGBB_toggle() HAL_GPIO_TogglePin(RGB_B_GPIO_Port, RGB_B_Pin)
#define RGBC_toggle() HAL_GPIO_TogglePin(RGB_C_GPIO_Port, RGB_C_Pin)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
