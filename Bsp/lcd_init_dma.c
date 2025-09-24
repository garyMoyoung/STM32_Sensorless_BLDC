#include "lcd_init_dma.h"
#include "spi.h"
#include "main.h"
#include "lcd_dma.h"
/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus_DMA(u8 dat) 
{	
	// LCD_CS_Clr();
	// HAL_SPI_Transmit(&hspi2,&dat,1,1000);//发送数据  
	// LCD_CS_Set();	
    LCD_WriteData_DMA(&dat, 1);
}
void LCD_WriteData_DMA(uint8_t* pData, uint16_t Size)
{
    LCD_CS_Clr();
    HAL_SPI_Transmit_DMA(&hspi2, pData, Size);
    while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){}
    LCD_CS_Set();
}

/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8_DMA(u8 dat)
{
	// LCD_Writ_Bus(dat);
    LCD_WriteData_DMA(&dat, 1);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA_DMA(u16 dat)
{
	// LCD_Writ_Bus(dat>>8);
	// LCD_Writ_Bus(dat);
    uint8_t data[2];
    data[0] = dat >> 8;
    data[1] = dat;
    LCD_WriteData_DMA(data, 2);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG_DMA(u8 dat)
{
	LCD_DC_Clr();//写命令
    LCD_WriteData_DMA(&dat, 1);
	// LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}

/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/

void LCD_Address_Set_DMA(u16 x1,u16 y1,u16 x2,u16 y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG_DMA(0x2a);//列地址设置
		LCD_WR_DATA_DMA(x1);
		LCD_WR_DATA_DMA(x2);
		LCD_WR_REG_DMA(0x2b);//行地址设置
		LCD_WR_DATA_DMA(y1+20);
		LCD_WR_DATA_DMA(y2+20);
		LCD_WR_REG_DMA(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG_DMA(0x2a);//列地址设置
		LCD_WR_DATA_DMA(x1);
		LCD_WR_DATA_DMA(x2);
		LCD_WR_REG_DMA(0x2b);//行地址设置
		LCD_WR_DATA_DMA(y1+20);
		LCD_WR_DATA_DMA(y2+20);
		LCD_WR_REG_DMA(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG_DMA(0x2a);//列地址设置
		LCD_WR_DATA_DMA(x1+20);
		LCD_WR_DATA_DMA(x2+20);
		LCD_WR_REG_DMA(0x2b);//行地址设置
		LCD_WR_DATA_DMA(y1);
		LCD_WR_DATA_DMA(y2);
		LCD_WR_REG_DMA(0x2c);//储存器写
	}
	else
	{
		LCD_WR_REG_DMA(0x2a);//列地址设置
		LCD_WR_DATA_DMA(x1+20);
		LCD_WR_DATA_DMA(x2+20);
		LCD_WR_REG_DMA(0x2b);//行地址设置
		LCD_WR_DATA_DMA(y1);
		LCD_WR_DATA_DMA(y2);
		LCD_WR_REG_DMA(0x2c);//储存器写
	}
}

void LCD_Init_DMA(void)
{
	hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    
    // 启用DMA
    __HAL_SPI_ENABLE(&hspi2);


	LCD_RES_Clr();//复位
    vTaskDelay(100);
	LCD_RES_Set();
	vTaskDelay(100);
	
	LCD_BLK_Set();//打开背光
    vTaskDelay(100);
	
	//************* Start Initial Sequence **********//
	LCD_WR_REG_DMA(0x11); //Sleep out 
	vTaskDelay(120);              //Delay 120ms 
	//************* Start Initial Sequence **********// 
	LCD_WR_REG_DMA(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8_DMA(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8_DMA(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8_DMA(0x70);
	else LCD_WR_DATA8_DMA(0xA0);

	LCD_WR_REG_DMA(0x3A);			
	LCD_WR_DATA8_DMA(0x05);

	LCD_WR_REG_DMA(0xB2);			
	LCD_WR_DATA8_DMA(0x0C);
	LCD_WR_DATA8_DMA(0x0C); 
	LCD_WR_DATA8_DMA(0x00); 
	LCD_WR_DATA8_DMA(0x33); 
	LCD_WR_DATA8_DMA(0x33); 			

	LCD_WR_REG_DMA(0xB7);			
	LCD_WR_DATA8_DMA(0x35);

	LCD_WR_REG_DMA(0xBB);			
	LCD_WR_DATA8_DMA(0x32); //Vcom=1.35V
					
	LCD_WR_REG_DMA(0xC2);
	LCD_WR_DATA8_DMA(0x01);

	LCD_WR_REG_DMA(0xC3);			
	LCD_WR_DATA8_DMA(0x15); //GVDD=4.8V  颜色深度
				
	LCD_WR_REG_DMA(0xC4);			
	LCD_WR_DATA8_DMA(0x20); //VDV, 0x20:0v

	LCD_WR_REG_DMA(0xC6);			
	LCD_WR_DATA8_DMA(0x0F); //0x0F:60Hz        	

	LCD_WR_REG_DMA(0xD0);			
	LCD_WR_DATA8_DMA(0xA4);
	LCD_WR_DATA8_DMA(0xA1); 

	LCD_WR_REG_DMA(0xE0);
	LCD_WR_DATA8_DMA(0xD0);   
	LCD_WR_DATA8_DMA(0x08);   
	LCD_WR_DATA8_DMA(0x0E);   
	LCD_WR_DATA8_DMA(0x09);   
	LCD_WR_DATA8_DMA(0x09);   
	LCD_WR_DATA8_DMA(0x05);   
	LCD_WR_DATA8_DMA(0x31);   
	LCD_WR_DATA8_DMA(0x33);   
	LCD_WR_DATA8_DMA(0x48);   
	LCD_WR_DATA8_DMA(0x17);   
	LCD_WR_DATA8_DMA(0x14);   
	LCD_WR_DATA8_DMA(0x15);   
	LCD_WR_DATA8_DMA(0x31);   
	LCD_WR_DATA8_DMA(0x34);   

	LCD_WR_REG_DMA(0xE1);     
	LCD_WR_DATA8_DMA(0xD0);   
	LCD_WR_DATA8_DMA(0x08);   
	LCD_WR_DATA8_DMA(0x0E);   
	LCD_WR_DATA8_DMA(0x09);   
	LCD_WR_DATA8_DMA(0x09);   
	LCD_WR_DATA8_DMA(0x15);   
	LCD_WR_DATA8_DMA(0x31);   
	LCD_WR_DATA8_DMA(0x33);   
	LCD_WR_DATA8_DMA(0x48);   
	LCD_WR_DATA8_DMA(0x17);   
	LCD_WR_DATA8_DMA(0x14);   
	LCD_WR_DATA8_DMA(0x15);   
	LCD_WR_DATA8_DMA(0x31);   
	LCD_WR_DATA8_DMA(0x34);
	LCD_WR_REG_DMA(0x21); 

	LCD_WR_REG_DMA(0x29);
} 








