/*
 * AS5600.h
 *
 *  Created on: Oct 19, 2024
 *      Author: yizha
 */

#ifndef _AS5600_H_
#define _AS5600_H_

/*
 * Includes
 */
#include "stm32f4xx_hal.h" // for I2C

/*
 * Defines
 */
#define AS5600_I2C_ADD (0x36 << 1) // left-shift by 1 for r/w bit
#define MAGNET_STATUS_REG 0x0B // 3:MHigh, 4:MLow, 5:MDetected; bit 1->true

/*
 * Registers
 */
#define RAW_ANGLE_MSB_REG 0x0C // RAW ANGLE 11:8
#define RAW_ANGLE_LSB_REG 0x0D // RAW ANGLE 7:0
#define ANGLE_MSB_REG 0x0E // ANGLE 11:8
#define ANGLE_LSB_REG 0x0F // ANGLE 7:0

/*
 * Constants
 */
#define BIT_TO_RAD 0.00153435538 // val / 12-bit * 2pi
#define MAX_RESOLUTION 4095 // 12-bit MAX
#define HALF_MAX_RESOLUTION 2047 // Half of 12-bit MAX

/*
 * Init status bitmasks
 */
#define MAGNET_OVERFLOW_MSK  (1 << 3)
#define MAGNET_UNDERFLOW_MSK (1 << 4)
#define MAGNET_OK_MSK         (1 << 5)
#define AS5600_READY_MSK      1

/*
 * Sensor struct
 */
typedef struct{
	// I2C handle
	I2C_HandleTypeDef *i2cHandle;

	// previous 12-bit angle
	uint16_t prev_raw_angle;
	uint8_t regdata[2];

	// total angle in radians
	float total_angle_rad;

	uint32_t prev_time_us;
} AS5600;

/*
 * Initialization
 * Pass the struct to each function as pointer
 */
uint8_t AS5600_Init(AS5600 *dev, I2C_HandleTypeDef *i2c_handle);
void AS5600_ZeroAngle(AS5600* dev);

/*
 * Read sensor value
 */
void AS5600_UpdateAngle_DMA(AS5600 *dev);
float AS5600_GetAngle(AS5600* dev);
uint16_t AS5600_GetRawAngle_Blocking(AS5600* dev);
uint16_t AS5600_GetRawAngle(AS5600* dev);
float AS5600_GetVelocity(AS5600* dev);

#endif /* INC_AS5600_H_ */
