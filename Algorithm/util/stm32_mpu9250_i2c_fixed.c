#include "main.h"
#include "stm32_mpu9250_i2c_fixed.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c3;

static void delay_us(uint32_t us)
{
    uint32_t delay = us * (SystemCoreClock / 1000000);
    while(delay--)
    {
        __asm("nop");  // Fixed: changed from asm("nop") to __asm("nop")
    }
}

int stm32_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[256];
    
    if(length > 255) return -1;
    
    buffer[0] = reg_addr;
    for(int i = 0; i < length; i++)
    {
        buffer[i + 1] = data[i];
    }
    
    status = HAL_I2C_Master_Transmit(&hi2c3, slave_addr << 1, buffer, length + 1, 1000);
    
    if(status != HAL_OK)
    {
        return -1;
    }
    
    return 0;
}

int stm32_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
    HAL_StatusTypeDef status;
    
    // Write register address
    status = HAL_I2C_Master_Transmit(&hi2c3, slave_addr << 1, &reg_addr, 1, 1000);
    if(status != HAL_OK)
    {
        return -1;
    }
    
    delay_us(10);
    
    // Read data
    status = HAL_I2C_Master_Receive(&hi2c3, slave_addr << 1, data, length, 1000);
    if(status != HAL_OK)
    {
        return -1;
    }
    
    return 0;
}

// Additional delay function for compatibility
static void delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms; i++)
    {
        delay_us(1000);
        __asm("nop");  // Fixed: changed from asm("nop") to __asm("nop")
    }
}