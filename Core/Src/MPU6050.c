#include "MPU6050.h"

#define DEG_TO_RAD									0.0174532925f
#define G_MPS2											9.80665f
#define DEFAULT_ACC_SCALE_LSB_G			16384.0f
#define DEFAULT_GYR_SCALE_LSB_DPS		131.0f

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
	imu->success_flag = 1;

	// XL data in m/s^2
	imu->acc_mps2[0] = 0.0f;
	imu->acc_mps2[1] = 0.0f;
	imu->acc_mps2[2] = 0.0f;

	// XL sensitivity according to datasheet in +/-2g range is 16384 LSB/g

	// Board 1 coefficients
	imu->acc_scale[0] = 0.9949237f;
	imu->acc_scale[1] = 0.9988063f;
	imu->acc_scale[2] = 0.9943320f;
	imu->acc_bias[0]  = -6.5000024f;
	imu->acc_bias[1]  = -67.4999988f;
	imu->acc_bias[2]  = -531.5000136f;
	imu->acc_yz_rot   = 0.0003194f;
	imu->acc_zy_rot   = -0.0021295f;
	imu->acc_zx_rot   = 0.0064480f;

	// Board 2 coefficients
//	imu->acc_scale[0] = 0.9949237f;
//	imu->acc_scale[1] = 0.9988063f;
//	imu->acc_scale[2] = 0.9943320f;
//	imu->acc_bias[0]  = -6.5000024f;
//	imu->acc_bias[1]  = -67.4999988f;
//	imu->acc_bias[2]  = -531.5000136f;
//	imu->acc_yz_rot   = 0.0003194f;
//	imu->acc_zy_rot   = -0.0021295f;
//	imu->acc_zx_rot   = 0.0064480f;

	// Temperature data
	imu->temp_C = 0.0f;

	// Gyro data in rps
	imu->gyr_rps[0] = 0.0f;
	imu->gyr_rps[1] = 0.0f;
	imu->gyr_rps[2] = 0.0f;

	// Gyro sensitivity according to datasheet in +/-250dps range is 131 LSB/dps
	imu->gyr_scale[0] = 1.0f;
	imu->gyr_scale[1] = 1.0f;
	imu->gyr_scale[2] = 1.0f;
	imu->gyr_bias[0]  = 0.0f;
	imu->gyr_bias[1]  = 0.0f;
	imu->gyr_bias[2]  = 0.0f;

	/* Check WHO_AM_I ID */

	HAL_StatusTypeDef status;
	uint8_t write_data;

	status = HAL_I2C_Mem_Read(imu->i2c_handle, imu->device_addr, MPU6050_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, imu->rx_buf, 1, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
	HAL_Delay(10);

	if(status != HAL_OK)
	{
		UART_println(&uart, "WHO_AM_I Read Failed.");
		imu->success_flag = 0;
		return 255; // TODO return err_num?
	}

	if (!(imu->rx_buf[0] == MPU6050_WHO_AM_I_ID || imu->rx_buf[0] == MPU6050_WHO_AM_I_ID_ALT))
	{
		UART_println(&uart, "WHO_AM_I ID Check Failed.");
		imu->success_flag = 0;
		return 255; // TODO return err_num?
	}

	/* Register Setup */

	write_data = 0x02;
	status = HAL_I2C_Mem_Write(imu->i2c_handle, imu->device_addr, MPU6050_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &write_data, 1, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
	if (status != HAL_OK)
		imu->success_flag = 0;
	HAL_Delay(5);

	write_data = 0x10;
	status = HAL_I2C_Mem_Write(imu->i2c_handle, imu->device_addr, MPU6050_REG_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &write_data, 1, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
	if (status != HAL_OK)
		imu->success_flag = 0;
	HAL_Delay(5);

	write_data = 0x01;
	status = HAL_I2C_Mem_Write(imu->i2c_handle, imu->device_addr, MPU6050_REG_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &write_data, 1, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
	if (status != HAL_OK)
		imu->success_flag = 0;
	HAL_Delay(5);

	write_data = 0x00;
	status = HAL_I2C_Mem_Write(imu->i2c_handle, imu->device_addr, MPU6050_REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &write_data, 1, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
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

uint8_t MPU6050_Read_DMA(MPU6050 *imu)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read_DMA(imu->i2c_handle, imu->device_addr, MPU6050_REG_DATA_START, I2C_MEMADD_SIZE_8BIT, imu->rx_buf, 14);
	if (status != HAL_OK)
	{
		imu->success_flag = 0;
		UART_log_error(&uart, status, "DMA Read Failed.");
	}

//	imu->dma_rx_flag = (status == HAL_OK);
	imu->dma_rx_flag = 1;
	imu->data_ready_flag = 0;

	return 0;
}

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
//	imu->acc_mps2[0] *= G_MPS2 / DEFAULT_ACC_SCALE_LSB_G;
//	imu->acc_mps2[1] *= G_MPS2 / DEFAULT_ACC_SCALE_LSB_G;
//	imu->acc_mps2[2] *= G_MPS2 / DEFAULT_ACC_SCALE_LSB_G;

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

//	imu->gyr_rps[0] *= 1.0f / DEFAULT_GYR_SCALE_LSB_DPS;
//	imu->gyr_rps[1] *= 1.0f / DEFAULT_GYR_SCALE_LSB_DPS;
//	imu->gyr_rps[2] *= 1.0f / DEFAULT_GYR_SCALE_LSB_DPS;
	imu->gyr_rps[0] *= DEG_TO_RAD / DEFAULT_GYR_SCALE_LSB_DPS;
	imu->gyr_rps[1] *= DEG_TO_RAD / DEFAULT_GYR_SCALE_LSB_DPS;
	imu->gyr_rps[2] *= DEG_TO_RAD / DEFAULT_GYR_SCALE_LSB_DPS;
}



HAL_StatusTypeDef MPU6050_Read_Register_Polling(MPU6050 *imu, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(imu->i2c_handle, imu->device_addr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
	return status;
}



HAL_StatusTypeDef MPU6050_Read_Registers_Polling(MPU6050 *imu, uint8_t reg, uint8_t *data, uint8_t length)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(imu->i2c_handle, imu->device_addr, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
	return status;
}



HAL_StatusTypeDef MPU6050_Write_Register_Polling(MPU6050 *imu, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(imu->i2c_handle, imu->device_addr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	while (HAL_I2C_IsDeviceReady(imu->i2c_handle, imu->device_addr, 1, HAL_MAX_DELAY) != HAL_OK);
	return status;
}
