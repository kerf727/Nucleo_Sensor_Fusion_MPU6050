/*
 * MPU6050.h
 *
 *  Created on: Jan 20, 2023
 *      Author: Kyle Erf
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32l4xx_hal.h" // Needed for I2C

#include "main.h"
#include "UART_API.h"

/*
 * Sensor Struct
 */

typedef struct
{
	// I2C
	I2C_HandleTypeDef *i2c_handle;
	uint8_t device_addr; // 0 for tied low, 1 for tied high

	// DMA
	uint8_t rx_buf[14];
	// TODO: is volatile keyword necessary? test to find out
	volatile uint8_t dma_rx_flag; // flag to indicate that data is being currently ready by DMA
	volatile uint8_t data_ready_flag;
	volatile uint8_t i2c_complete;
	volatile uint8_t success_flag;

	// XL data in m/s^2
	float acc_mps2[3];

	float acc_scale[3];
	float acc_bias[3];
	float acc_yz_rot;
	float acc_zy_rot;
	float acc_zx_rot;

	// Temperature data
	float temp_C;

	// Gyro data in rps
	float gyr_rps[3];

	float gyr_scale[3];
	float gyr_bias[3];

} MPU6050;

/*
 * Defines
 */

#define MPU6050_I2C_ADDR_AD0_0		(0b1101000 << 1) // pin AD0 is low
#define MPU6050_I2C_ADDR_AD0_1		(0b1101001 << 1) // pin AD0 is high

#define MPU6050_WHO_AM_I_ID				0x68 // Board 1 has expected ID of 0x68
#define MPU6050_WHO_AM_I_ID_ALT		0x72 // Board 2 has unexpected ID of 0x72

/*
 * Registers
 */

#define MPU6050_REG_CONFIG				0x1A
#define MPU6050_REG_GYRO_CONFIG		0x1B
#define MPU6050_REG_ACCEL_CONFIG	0x1C

#define MPU6050_REG_INT_PIN_CFG		0x37
#define MPU6050_REG_INT_ENABLE		0x38
#define MPU6050_REG_INT_STATUS		0x3A

#define MPU6050_REG_DATA_START		0x3B

#define MPU6050_REG_ACCEL_XOUT_H	0x3B
#define MPU6050_REG_ACCEL_XOUT_L	0x3C
#define MPU6050_REG_ACCEL_YOUT_H	0x3D
#define MPU6050_REG_ACCEL_YOUT_L	0x3E
#define MPU6050_REG_ACCEL_ZOUT_H	0x3F
#define MPU6050_REG_ACCEL_ZOUT_L	0x40

#define MPU6050_REG_TEMP_OUT_H		0x41
#define MPU6050_REG_TEMP_OUT_L		0x42

#define MPU6050_REG_GYRO_XOUT_H		0x43
#define MPU6050_REG_GYRO_XOUT_L		0x44
#define MPU6050_REG_GYRO_YOUT_H		0x45
#define MPU6050_REG_GYRO_YOUT_L		0x46
#define MPU6050_REG_GYRO_ZOUT_H		0x47
#define MPU6050_REG_GYRO_ZOUT_L		0x48

#define MPU6050_REG_PWR_MGMT_1		0x6B
#define MPU6050_REG_PWR_MGMT_2		0x6C
#define MPU6050_REG_WHO_AM_I			0x75

/*
 * Initialization
 */

uint8_t MPU6050_Init(MPU6050 *imu, I2C_HandleTypeDef *i2cHandle, uint8_t AD0_Pin_Value);

/*
 * Data acquisition (DMA)
 */

uint8_t MPU6050_Read_DMA(MPU6050 *imu);
void MPU6050_Process_Data(MPU6050 *imu);

/*
 * Low-level functions
 */

HAL_StatusTypeDef MPU6050_Read_Register_Polling(MPU6050 *imu, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MPU6050_Read_Registers_Polling(MPU6050 *imu, uint8_t reg, uint8_t *data, uint8_t length);
HAL_StatusTypeDef MPU6050_Write_Register_Polling(MPU6050 *imu, uint8_t reg, uint8_t *data);

#endif /* INC_MPU6050_H_ */
