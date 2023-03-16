#include "MPU6050.h"

#define DEG_TO_RAD									0.0174532925f
#define G_MPS2											9.80665f
#define DEFAULT_ACC_SCALE_LSB_G			16384.0f
#define DEFAULT_GYR_SCALE_LSB_DPS		131.0f



/*
 * @brief Initialize MPU6050
 * @param MPU6050 IMU struct
 * @param I2C handle
 * @param AD0 Pin Value (changes I2C address depending on whether it's tied low or high)
 * @retval Error code
 */
uint8_t MPU6050_Init(MPU6050 *imu, I2C_HandleTypeDef *i2c_handle, uint8_t AD0_Pin_Value)
{
	/* Set Struct Parameters */

	// I2C
	imu->i2c_handle = i2c_handle;
	imu->device_addr = AD0_Pin_Value ? MPU6050_I2C_ADDR_AD0_1 : MPU6050_I2C_ADDR_AD0_0;

	// DMA
	for (uint8_t i = 0; i < 14; i++)
	{
		imu->rx_buf[i] = 0;
	}
	imu->dma_rx_flag = 0;
	imu->data_ready_flag = 0;
	imu->i2c_complete = 0;

	imu->success_flag = 1;

	// XL data in m/s^2
	imu->acc_mps2[0] = 0.0f;
	imu->acc_mps2[1] = 0.0f;
	imu->acc_mps2[2] = 0.0f;

	// Temperature data
	imu->temp_C = 0.0f; 
	
	// Gyro data in rps
	imu->gyr_rps[0] = 0.0f;
	imu->gyr_rps[1] = 0.0f;
	imu->gyr_rps[2] = 0.0f;

	// XL sensitivity according to datasheet in +/-2g range is 16384 LSB/g
	imu->acc_scale[0] = 0.9948657f;
	imu->acc_scale[1] = 0.9986902f;
	imu->acc_scale[2] = 0.9947902f;
	imu->acc_bias[0]  = -10.8274112f;  // LSBs
	imu->acc_bias[1]  = -66.3366318f;  // LSBs
	imu->acc_bias[2]  = -549.9550669f; // LSBs
	imu->acc_yz_rot   = -0.0003177f;
	imu->acc_zy_rot   = -0.0027145f;
	imu->acc_zx_rot   = 0.0060455f;

	// Gyro sensitivity according to datasheet in +/-250dps range is 131 LSB/dps
	imu->gyr_scale[0] = 1.0f;
	imu->gyr_scale[1] = 1.0f;
	imu->gyr_scale[2] = 1.0f;
	imu->gyr_bias[0]  = -289.752f;    // LSBs
	imu->gyr_bias[1]  = 249.255f;     // LSBs
	imu->gyr_bias[2]  = 7.745f;       // LSBs

	/* Check WHO_AM_I ID */

	HAL_StatusTypeDef status;
	uint8_t write_data;

	status = MPU6050_Read_Register_Polling(imu, MPU6050_REG_WHO_AM_I, imu->rx_buf);
	HAL_Delay(10);

	if(status != HAL_OK)
	{
		UART_println(&uart, "WHO_AM_I Read Failed.");
		imu->success_flag = 0;
		return 255;
	}

	if (!(imu->rx_buf[0] == MPU6050_WHO_AM_I_ID || imu->rx_buf[0] == MPU6050_WHO_AM_I_ID_ALT))
	{
		UART_println(&uart, "WHO_AM_I ID Check Failed.");
		imu->success_flag = 0;
		return 255;
	}

	/* Register Setup */

	// DLPF_CONFIG = 2: ODR = 1kHz, XL BW = 94Hz, Gyro BW = 98Hz
	write_data = 0x02;
	status = MPU6050_Write_Register_Polling(imu, MPU6050_REG_CONFIG, &write_data);
	if (status != HAL_OK)
		imu->success_flag = 0;
	HAL_Delay(5);

	// INT_RD_CLEAR = 1: interrupt status bits are cleared on any read operation
	write_data = 0x10;
	status = MPU6050_Write_Register_Polling(imu, MPU6050_REG_INT_PIN_CFG, &write_data);
	if (status != HAL_OK)
		imu->success_flag = 0;
	HAL_Delay(5);

	// DATA_RDY_EN = 1: enables the data ready interrupt
	write_data = 0x01;
	status = MPU6050_Write_Register_Polling(imu, MPU6050_REG_INT_ENABLE, &write_data);
	if (status != HAL_OK)
		imu->success_flag = 0;
	HAL_Delay(5);

	// SLEEP = 0: disable sleep mode
	write_data = 0x00;
	status = MPU6050_Write_Register_Polling(imu, MPU6050_REG_PWR_MGMT_1, &write_data);
	if (status != HAL_OK)
		imu->success_flag = 0;
	HAL_Delay(5);

	if (imu->success_flag)
	{
		UART_println(&uart, "Initialization Succeeded.");
	}
	else
	{
		UART_println(&uart, "Initialization Failed.");
	}

	return 0;
}



/*
 * @brief Read Sensor Data with I2C DMA
 * @param MPU6050 IMU struct
 * @retval Error code
 */
uint8_t MPU6050_Read_DMA(MPU6050 *imu)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read_DMA(imu->i2c_handle, imu->device_addr, MPU6050_REG_DATA_START, I2C_MEMADD_SIZE_8BIT, imu->rx_buf, 14);
	if (status != HAL_OK)
	{
		imu->success_flag = 0;
		UART_log_error(&uart, status, "DMA Read Failed.");
	}

	imu->dma_rx_flag = 1;
	imu->data_ready_flag = 0;

	return 0;
}




/*
 * @brief Process Sensor Data
 * @param MPU6050 IMU struct
 * @retval None
 */
void MPU6050_Process_Data(MPU6050 *imu)
{
	/* Read Accelerometer */

	int16_t acc_raw_signed[3];
	acc_raw_signed[0] = (int16_t) ((imu->rx_buf[0] << 8) | imu->rx_buf[1]); // X-axis
	acc_raw_signed[1] = (int16_t) ((imu->rx_buf[2] << 8) | imu->rx_buf[3]); // Y-axis
	acc_raw_signed[2] = (int16_t) ((imu->rx_buf[4] << 8) | imu->rx_buf[5]); // Z-axis

	// Accelerometer model: a_cal = T * K * (a_raw + b)
	float K_a_b[3];
	K_a_b[0] = imu->acc_scale[0] * ((float) acc_raw_signed[0] + imu->acc_bias[0]);
	K_a_b[1] = imu->acc_scale[1] * ((float) acc_raw_signed[1] + imu->acc_bias[1]);
	K_a_b[2] = imu->acc_scale[2] * ((float) acc_raw_signed[2] + imu->acc_bias[2]);

	imu->acc_mps2[0] = K_a_b[0] - imu->acc_yz_rot * K_a_b[1] + imu->acc_zy_rot * K_a_b[2];
	imu->acc_mps2[1] = K_a_b[1] - imu->acc_zx_rot * K_a_b[2];
	imu->acc_mps2[2] = K_a_b[2];

	imu->acc_mps2[0] *= 1.0f / DEFAULT_ACC_SCALE_LSB_G;
	imu->acc_mps2[1] *= 1.0f / DEFAULT_ACC_SCALE_LSB_G;
	imu->acc_mps2[2] *= 1.0f / DEFAULT_ACC_SCALE_LSB_G;
//	imu->acc_mps2[0] *= G_MPS2;
//	imu->acc_mps2[1] *= G_MPS2;
//	imu->acc_mps2[2] *= G_MPS2;

	/* Read Temperature */

	int16_t temp_raw_signed = (int16_t) ((imu->rx_buf[6] << 8) | imu->rx_buf[7]);
	imu->temp_C = (float) temp_raw_signed / 340.0f + 36.53f;

	/* Read Gyroscope */
	int16_t gyr_raw_signed[3];
	gyr_raw_signed[0] = (int16_t) ((imu->rx_buf[ 8] << 8) | imu->rx_buf[ 9]); // X-axis
	gyr_raw_signed[1] = (int16_t) ((imu->rx_buf[10] << 8) | imu->rx_buf[11]); // Y-axis
	gyr_raw_signed[2] = (int16_t) ((imu->rx_buf[12] << 8) | imu->rx_buf[13]); // Z-axis

	imu->gyr_rps[0] = imu->gyr_scale[0] * ((float) gyr_raw_signed[0] + imu->gyr_bias[0]);
	imu->gyr_rps[1] = imu->gyr_scale[1] * ((float) gyr_raw_signed[1] + imu->gyr_bias[1]);
	imu->gyr_rps[2] = imu->gyr_scale[2] * ((float) gyr_raw_signed[2] + imu->gyr_bias[2]);

	imu->gyr_rps[0] *= DEG_TO_RAD / DEFAULT_GYR_SCALE_LSB_DPS;
	imu->gyr_rps[1] *= DEG_TO_RAD / DEFAULT_GYR_SCALE_LSB_DPS;
	imu->gyr_rps[2] *= DEG_TO_RAD / DEFAULT_GYR_SCALE_LSB_DPS;
}



/*
 * @brief I2C Read Register via Polling
 * @param MPU6050 IMU struct
 * @param MPU6050 register address byte
 * @param MPU6050 read data value byte
 * @retval HAL Status
 */
HAL_StatusTypeDef MPU6050_Read_Register_Polling(MPU6050 *imu, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(imu->i2c_handle, imu->device_addr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
	return status;
}



/*
 * @brief I2C Read Multiple Registers via Polling
 * @param MPU6050 IMU struct
 * @param MPU6050 register address byte
 * @param MPU6050 read data value byte
 * @param length: number of consecutive addresses to read
 * @retval HAL Status
 */
HAL_StatusTypeDef MPU6050_Read_Registers_Polling(MPU6050 *imu, uint8_t reg, uint8_t *data, uint8_t length)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(imu->i2c_handle, imu->device_addr, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
	return status;
}



/*
 * @brief I2C Write Register via Polling
 * @param MPU6050 IMU struct
 * @param MPU6050 register address byte
 * @param MPU6050 write data value byte
 * @retval HAL Status
 */
HAL_StatusTypeDef MPU6050_Write_Register_Polling(MPU6050 *imu, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(imu->i2c_handle, imu->device_addr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
	return status;
}
