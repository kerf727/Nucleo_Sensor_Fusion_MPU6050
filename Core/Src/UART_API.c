#include "UART_API.h"

void UART_Init(UART_API *uart, UART_HandleTypeDef *huart, uint16_t max_len)
{
	uart->huart = huart;
	uart->log_buf[0] = '\0'; // null terminate
	uart->log_buf_len = 1;
	uart->max_len = max_len;
}



void UART_print_arg(UART_API *uart, char *msg, uint32_t arg)
{
	if (strlen(msg) < uart->max_len)
	{
		uart->log_buf_len = sprintf(uart->log_buf, msg, arg);
		HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
	}
	else
	{
		uart->log_buf_len = sprintf(uart->log_buf, "Message needs to be less than %d characters long.\r\n", uart->max_len);
		HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
	}
}


/* Not working
void UART_println_args(UART_API *uart, char *msg, uint32_t *args, uint8_t num_args)
{
	size_t msg_len = strlen(msg);
	if (msg_len + 3 < uart->max_len)
	{
		// Concatenate CR+LF and null termination to msg
//		strncpy(uart->log_buf, msg, msg_len);
//		strncpy(uart->log_buf + msg_len, "\r\n", 3);

		char new_msg[msg_len + 3];
		strncpy(new_msg, msg, msg_len + 1);		strncat(new_msg, "\r\n", 3);

		if (num_args == 1)
			uart->log_buf_len = sprintf(uart->log_buf, new_msg, *args);
		else if (num_args == 2)
		{
			uart->log_buf_len = sprintf(uart->log_buf, new_msg, args[0], args[1]);
		}
		else
		{
			uart->log_buf_len = sprintf(uart->log_buf, "Must use 2 or less args.\r\n");
		}

		HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
	}
	else
	{
		uart->log_buf_len = sprintf(uart->log_buf, "Message needs to be less than %d characters long.\r\n", uart->max_len);
		HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
	}
}
*/


void UART_print(UART_API *uart, char *msg)
{
	if (strlen(msg) < uart->max_len)
	{
		uart->log_buf_len = sprintf(uart->log_buf, msg);
		HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
	}
	else
	{
		uart->log_buf_len = sprintf(uart->log_buf, "Message needs to be less than %d characters long.\r\n", uart->max_len);
		HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
	}
}



void UART_println(UART_API *uart, char *msg)
{
	if (strlen(msg) < uart->max_len)
	{
		uart->log_buf_len = sprintf(uart->log_buf, "%s\r\n", msg);
		HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
	}
	else
	{
		uart->log_buf_len = sprintf(uart->log_buf, "Message needs to be less than %d characters long.\r\n", uart->max_len);
		HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
	}
}



void UART_log_error(UART_API *uart, HAL_StatusTypeDef status, char *error_msg)
{
	switch(status)
	{
	case(HAL_OK):
		uart->log_buf_len = sprintf(uart->log_buf, "%s\r\n", error_msg);
		break;
	case(HAL_ERROR):
		uart->log_buf_len = sprintf(uart->log_buf, "%s Status: Error\r\n", error_msg);
		break;
	case(HAL_BUSY):
		uart->log_buf_len = sprintf(uart->log_buf, "%s Status: Busy\r\n", error_msg);
		break;
	case(HAL_TIMEOUT):
		uart->log_buf_len = sprintf(uart->log_buf, "%s Status: Timeout\r\n", error_msg);
		break;
	default:
		uart->log_buf_len = sprintf(uart->log_buf, "Unknown Status Value\r\n");
	}
	HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
}



void UART_log_value(UART_API *uart, char *value_name, uint32_t value)
{
	uart->log_buf_len = sprintf(uart->log_buf, "%s: %lu\r\n", value_name, value);
	HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
}



void UART_print_float(UART_API *uart, float value)
{
	uart->log_buf_len = sprintf(uart->log_buf, "%.3f\r\n", value);
	HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
}



void UART_print_sensor(UART_API *uart, float *sensor)
{
	uart->log_buf_len = sprintf(uart->log_buf, "%8.3f,%8.3f,%8.3f\r\n",
			sensor[0], sensor[1], sensor[2]);
	HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
}



void UART_print_two_sensors(UART_API *uart, float *sensor1, float *sensor2)
{
	uart->log_buf_len = sprintf(uart->log_buf, "%8.3f,%8.3f,%8.3f    %8.3f,%8.3f,%8.3f\r\n",
			sensor1[0], sensor1[1], sensor1[2],
			sensor2[0], sensor2[1], sensor2[2]);
	HAL_UART_Transmit(uart->huart, (uint8_t *)uart->log_buf, uart->log_buf_len, HAL_MAX_DELAY);
}
