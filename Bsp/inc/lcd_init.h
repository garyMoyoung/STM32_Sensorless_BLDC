#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "main.h"

#define USE_HORIZONTAL 0  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif


//-----------------LCD端口定义---------------- 

//#define LCD_SCLK_Clr() GPIO_ResetBits(GPIOG,GPIO_Pin_12)//SCL=SCLK
//#define LCD_SCLK_Set() GPIO_SetBits(GPIOG,GPIO_Pin_12)

//#define LCD_MOSI_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_5)//SDA=MOSI
//#define LCD_MOSI_Set() GPIO_SetBits(GPIOD,GPIO_Pin_5)

//#define LCD_RES_Clr()  GPIO_ResetBits(GPIOD,GPIO_Pin_4)//RES
//#define LCD_RES_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_4)

//#define LCD_DC_Clr()   GPIO_ResetBits(GPIOD,GPIO_Pin_15)//DC
//#define LCD_DC_Set()   GPIO_SetBits(GPIOD,GPIO_Pin_15)
// 		     
//#define LCD_CS_Clr()   GPIO_ResetBits(GPIOD,GPIO_Pin_1)//CS
//#define LCD_CS_Set()   GPIO_SetBits(GPIOD,GPIO_Pin_1)

//#define LCD_BLK_Clr()  GPIO_ResetBits(GPIOE,GPIO_Pin_8)//BLK
//#define LCD_BLK_Set()  GPIO_SetBits(GPIOE,GPIO_Pin_8)

#define LCD_RES_Clr()  HAL_GPIO_WritePin(GPIOF,TFT_RES_Pin, GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(GPIOF,TFT_RES_Pin, GPIO_PIN_SET)
 
#define LCD_DC_Clr()   HAL_GPIO_WritePin(GPIOF,TFT_DC_Pin, GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(GPIOF,TFT_DC_Pin, GPIO_PIN_SET)
         
#define LCD_CS_Clr()   HAL_GPIO_WritePin(GPIOF,TFT_CS_Pin, GPIO_PIN_RESET)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(GPIOF,TFT_CS_Pin, GPIO_PIN_SET)
 
#define LCD_BLK_Clr()  HAL_GPIO_WritePin(GPIOG,TFT_BL_Pin, GPIO_PIN_RESET)//BLK
#define LCD_BLK_Set()  HAL_GPIO_WritePin(GPIOG,TFT_BL_Pin, GPIO_PIN_SET)

void LCD_WriteData_DMA(uint8_t* pData, uint16_t Size); //LCD串行数据写入函数
void LCD_WriteData_DMA(uint8_t* pData, uint16_t Size);//LCD串行数据写入函数
void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void LCD_Init(void);//LCD初始化





#endif




