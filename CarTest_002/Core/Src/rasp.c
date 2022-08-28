/*
 * rasp.c
 *
 *  Created on: Aug 18, 2022
 *      Author: DELL
 */

#include "rasp.h"
#include <usart.h>
#include <string.h>
#include "motor.h"

uint8_t rx_6;

void Rasp_Init() {
	HAL_UART_Receive_IT(&huart6, &rx_6, 1);
}

void Rasp_UART_CallBack() {

	uint8_t rx_temp;
	rx_temp = rx_6 - '0';
	Move(rx_temp);

	HAL_UART_Transmit(&huart3, &rx_6, 1, 100);
	HAL_UART_Receive_IT(&huart6, &rx_6, 1);
}

