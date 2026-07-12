#include "AS5600.h"
#include "timer_utils.h"

static AS5600 *as5600_dma_dev = NULL;

#define AS5600_I2C_TIMEOUT_MS  2U

static void AS5600_CancelPendingTransfer(AS5600 *dev)
{
	I2C_HandleTypeDef *hi2c;

	if (dev == NULL)
	{
		return;
	}

	hi2c = dev->i2cHandle;
	as5600_dma_dev = NULL;

	if ((hi2c == NULL) || (hi2c->State == HAL_I2C_STATE_READY))
	{
		return;
	}

	if (hi2c->hdmarx != NULL)
	{
		(void)HAL_DMA_Abort(hi2c->hdmarx);
	}
	if (hi2c->hdmatx != NULL)
	{
		(void)HAL_DMA_Abort(hi2c->hdmatx);
	}

	hi2c->State = HAL_I2C_STATE_READY;
	hi2c->Mode = HAL_I2C_MODE_NONE;
	hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
}

/*
 * Static low level functions to read/write to registers
 */
__STATIC_INLINE HAL_StatusTypeDef AS5600_ReadRegister(AS5600 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, AS5600_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

__STATIC_INLINE HAL_StatusTypeDef AS5600_ReadRegisters(AS5600 *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, AS5600_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100);
}

__STATIC_INLINE HAL_StatusTypeDef AS5600_ReadRegister_DMA(AS5600 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read_DMA(dev->i2cHandle, AS5600_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, 1);
}

__STATIC_INLINE HAL_StatusTypeDef AS5600_ReadRegisters_DMA(AS5600 *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read_DMA(dev->i2cHandle, AS5600_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, length);
}

__STATIC_INLINE HAL_StatusTypeDef AS5600_CheckSensor(AS5600* dev, uint32_t trials)
{
	return HAL_I2C_IsDeviceReady(dev->i2cHandle, AS5600_I2C_ADD, trials, 1000);
}

/*
 * Utility functions
 */

/*
 * @brief Zeroes the sensor value.
 * @param[in] AS5600* sensor
 */
void AS5600_ZeroAngle(AS5600* dev)
{
	dev->total_angle_rad = 0;
	dev->prev_raw_angle = AS5600_GetRawAngle_Blocking(dev);
}

/*
 * Initialization
 * Pass the struct to each function as pointer
 */
uint8_t AS5600_Init(AS5600* dev, I2C_HandleTypeDef* i2c_handle)
{
	uint8_t init_status = 0;

	/* Set struct params */
	dev->i2cHandle = i2c_handle;
	dev->total_angle_rad = 0;
	dev->prev_time_us = micros();
	dev->regdata[0] = 0;
	dev->regdata[1] = 0;
	dev->angle_valid = 0;

	HAL_StatusTypeDef sensor_status = AS5600_CheckSensor(dev, 10);

	if(sensor_status != HAL_OK)
	{
		return (init_status | AS5600_READY_MSK);
	}

	dev->prev_raw_angle = AS5600_GetRawAngle_Blocking(dev);
	dev->total_angle_rad = dev->prev_raw_angle * BIT_TO_RAD;
	dev->angle_valid = 1;

	/* Check magnet strength */
	uint8_t magnet_status = 0;

	AS5600_ReadRegister(dev, MAGNET_STATUS_REG, &magnet_status);
	if((magnet_status & MAGNET_OK_MSK) == 0)
	{
		return (init_status | magnet_status);
	}

	return init_status;
}

/*
 * @brief Blocking function to read AS5600 sensor raw angle
 */
uint16_t AS5600_GetRawAngle_Blocking(AS5600* dev)
{
	HAL_StatusTypeDef status = AS5600_ReadRegisters(dev, RAW_ANGLE_MSB_REG, dev->regdata, 2);

	if(status != HAL_OK)
	{
		return 0;
	}

	uint16_t raw_angle = (((dev->regdata[0] & 0x0F) << 8) | dev->regdata[1]);
	return raw_angle;
}

static void AS5600_ProcessRawAngle(AS5600 *dev)
{
	uint16_t raw_angle;
	int16_t delta;

	if(dev == NULL) {
		return;
	}

	raw_angle = (((dev->regdata[0] & 0x0F) << 8) | dev->regdata[1]);
	if(dev->angle_valid == 0) {
		dev->prev_raw_angle = raw_angle;
		dev->total_angle_rad = raw_angle * BIT_TO_RAD;
		dev->angle_valid = 1;
		return;
	}

	delta = raw_angle - dev->prev_raw_angle;

	if(delta > HALF_MAX_RESOLUTION) {
		dev->total_angle_rad -= (MAX_RESOLUTION - delta) * BIT_TO_RAD;
	}
	else if(delta < -HALF_MAX_RESOLUTION) {
		dev->total_angle_rad += (MAX_RESOLUTION + delta) * BIT_TO_RAD;
	}
	else {
		dev->total_angle_rad += delta * BIT_TO_RAD;
	}

	dev->prev_raw_angle = raw_angle;
}

/*
 * @brief Read the AS5600 angle through blocking I2C and update the accumulator.
 */
void AS5600_UpdateAngle(AS5600 *dev)
{
	if((dev == NULL) || (dev->i2cHandle == NULL)) {
		return;
	}

	AS5600_CancelPendingTransfer(dev);

	if(HAL_I2C_Mem_Read(dev->i2cHandle, AS5600_I2C_ADD, RAW_ANGLE_MSB_REG,
	                    I2C_MEMADD_SIZE_8BIT, dev->regdata, 2, AS5600_I2C_TIMEOUT_MS) == HAL_OK) {
		AS5600_ProcessRawAngle(dev);
	}
}

void AS5600_UpdateAngle_DMA(AS5600 *dev)
{
	if((dev == NULL) || (dev->i2cHandle == NULL) || (as5600_dma_dev != NULL)) {
		return;
	}

	as5600_dma_dev = dev;
	if(AS5600_ReadRegisters_DMA(dev, RAW_ANGLE_MSB_REG, dev->regdata, 2) != HAL_OK) {
		as5600_dma_dev = NULL;
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if((as5600_dma_dev != NULL) && (hi2c == as5600_dma_dev->i2cHandle)) {
		AS5600_ProcessRawAngle(as5600_dma_dev);
		as5600_dma_dev = NULL;
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if((as5600_dma_dev != NULL) && (hi2c == as5600_dma_dev->i2cHandle)) {
		as5600_dma_dev = NULL;
	}
}

float AS5600_GetAngle(AS5600* dev)
{
	return dev->total_angle_rad;
}

uint16_t AS5600_GetRawAngle(AS5600* dev)
{
	return dev->prev_raw_angle;
}

/* @brief Returns the rate of change of sensor angle (velocity)
 * @param[in] AS5600* sensor
 */
float AS5600_GetVelocity(AS5600* dev)
{
	uint32_t now_us = micros();

	/* Save previous angle */
	float prev_angle = dev->total_angle_rad;
    printf("Previous angle: %.4f rad\n", prev_angle);

	AS5600_UpdateAngle(dev);
	/* Calculate time delta */
	float time_delta_s = (now_us - dev->prev_time_us) * 0.000001f;
	time_delta_s = (time_delta_s > 0.1) ? 0.0001f : time_delta_s;

	/* Calculate angle delta */
	float current_angle = AS5600_GetAngle(dev);
    printf("Current angle: %.4f rad\n", current_angle);

	float angle_delta = current_angle - prev_angle;
	
	/* Update sensor timestamp */
	dev->prev_time_us = now_us;
	printf("angle_delta:%.4f,time_delta_s:%.4f\n",angle_delta,time_delta_s);
	
	return angle_delta / time_delta_s;
}

// 从弧度/秒转换为RPM (Revolutions Per Minute)
float rad_sec_to_rpm(float rad_sec) {
    return rad_sec * 60.0f / (2.0f * PI);
}

// 从弧度/秒转换为Hz (Revolutions Per Second)
float rad_sec_to_hz(float rad_sec) {
    return rad_sec / (2.0f * PI);
}

