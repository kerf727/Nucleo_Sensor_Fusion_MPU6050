/*
 * UART_API.h
 *
 *  Created on: Jan 20, 2023
 *      Author: Kyle Erf
 */

#ifndef INC_UART_API_H_
#define INC_UART_API_H_

#include <stdio.h>
#include <string.h>

#include "main.h" // Needed for UART_API instance
#include "stm32l4xx_hal.h" // Needed for HAL_StatusTypeDef

typedef struct UART_API
{
	UART_HandleTypeDef *huart;

	char log_buf[128];
	uint16_t log_buf_len;
	uint16_t max_len;

} UART_API;

void UART_Init(UART_API *uart, UART_HandleTypeDef *huart, uint16_t max_len);

void UART_print(UART_API *uart, char *msg);
void UART_println(UART_API *uart, char *msg);
void UART_print_arg(UART_API *uart, char *msg, uint32_t arg);
//void UART_print_args(UART_API *uart, char *msg, uint32_t *args, uint8_t num_args);
//void UART_println_args(UART_API *uart, char *msg, uint32_t *args, uint8_t num_args);

void UART_log_error(UART_API *uart, HAL_StatusTypeDef status, char *error_msg);

void UART_log_value(UART_API *uart, char *value_name, uint32_t value);

// sensor = array[3]
void UART_print_sensor(UART_API *uart, float *sensor);
void UART_print_two_sensors(UART_API *uart, float *sensor1, float *sensor2);

#endif // INC_UART_API_H_
