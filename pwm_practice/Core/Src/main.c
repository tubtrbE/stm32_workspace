/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	// APB1 == 90[mhz]
	// prescaler == 30-1
	N = 0,
	C = 11762,
	D = 10469,
	E = 9318,
	F = 8791,
	G = 7825,
	A = 6966,
	B = 6200,
} _PITCH;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t pitch_main;
uint32_t volume_main;
uint32_t time_main;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void note(uint32_t pitch, uint32_t volume, uint32_t time, uint32_t octave);
uint32_t pitch_change (char pitch_text);
uint32_t octave_change (char octave_text);
uint32_t temp_change (char temp_text);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart3, &ch, 1, 100);
	return ch;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_RTC_Init();
	MX_TIM3_Init();
	MX_USART3_UART_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	//이무진 신호등 1절

	char *verse1[] = {

	///////////////////////////////////////////////////////////
			"N5N", "C5N", "D5N", "E5N", "C5N", "G4N", "C5N", "G4N", "C5N",
			"C5N", "D5N", "E5N", "E5N", "N5N", "N5N", "C5N", "D5N", "E5N",
			"C5N", "G4N", "C5N", "G4N", "C5N", "C5N", "E5N", "F5N", "E5N",
			"D5N", "C5N", "C5N",
	///////////////////////////////////////////////////////////
			};

	int verse1_time[] = { 4, 8, 8, 4, 8, 8, 8, 16, 16, 32, 16, 32, 4, 4, 4, 8,
			8, 4, 8, 8, 8, 16, 16, 16, 6, 16, 16, 16, 8, 16 };

		/* USER CODE END 2 */

		/* Infinite loop */
		/* USER CODE BEGIN WHILE */

	while (1) {
		int i = 0;
		int j = 0;
		int count_time = 0;
		while (verse1[i][j] != 0) {
			for (j = 0; j < 3; j++) {
				uint32_t pitch = 0;
				uint32_t octave = 0;
				int time = 0;

				pitch = pitch_change(verse1[i][0]);
				octave = pitch_change(verse1[i][1]);
				time = verse1_time[count_time];
				count_time++;

				note(pitch, 200, 2000/time, octave);

				printf("%c\n", verse1[i][0]);
				printf("%c\n", verse1[i][1]);
				printf("%c\n", verse1[i][2]);
				printf("%c\n", verse1_time[count_time]);
			}
			i++;
			printf("\r\n");

		}
//	  note(C, 2, 1000, 4); //도
//	  note(D, 2, 1000, 4); //레
//	  note(E, 2, 1000, 4); //미
//	  note(F, 2, 1000, 4); //파
//	  note(G, 2, 1000, 4); //솔
//	  note(A, 2, 1000, 4); //라
//	  note(B, 2, 1000, 4); //시

	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* USART3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
void note(uint32_t pitch, uint32_t volume, uint32_t time, uint32_t octave) {

	uint32_t start_tick = 0;
	uint32_t cur_tick = 0;
	uint32_t tick_gap = 0;

	// 1/1000 is enough to turn off the volume
	if (pitch == 0) {
		volume = 2000;
	}

	// avoid error(ARR == CCR)
	if (volume <= 2) {
		volume = 2;
	}

	// setting the octave
	if (octave != 4) {
		if (octave < 4) {
			for (int i = 0; i < 4 - octave; i++) {
				pitch *= 2;
			}
		} else {
			for (int i = 0; i < octave - 4; i++) {
				pitch /= 2;
			}
		}
	}

	TIM3->ARR = pitch;
	TIM3->CCR1 = pitch / volume;

	start_tick = HAL_GetTick();
	tick_gap = 0;
	while (tick_gap >= 0) {
		cur_tick = HAL_GetTick();
		tick_gap = cur_tick - start_tick;
		TIM3->CCR1 = pitch / volume;

		if (tick_gap >= time) {
			volume = 2;
			tick_gap = 0;
			break;
		}
//		printf("volume : %d\r\n", volume);
//		printf("tick_gap : %d\r\n", tick_gap);
		if (tick_gap % 5 == 0 || tick_gap % 4 == 0 || tick_gap % 3 == 0) {
			volume += 2;
		}
	}
}
uint32_t pitch_change (char pitch_text) {
	if (pitch_text == 'N') {
		return 0;
	}
	else if (pitch_text == 'C') {
		return C;
	}
	else if (pitch_text == 'D') {
		return D;
	}
	else if (pitch_text == 'E') {
		return E;
	}
	else if (pitch_text == 'F') {
		return F;
	}
	else if (pitch_text == 'G') {
		return G;
	}
	else if (pitch_text == 'A') {
		return A;
	}
	else if (pitch_text == 'B') {
		return B;
	}
	else {
		return 0;
	}
}
uint32_t octave_change (char octave_text) {
	return octave_text - 'A';
}
uint32_t temp_change (char temp_text) {
	if (temp_text == 'N') {
		return 0;
	}
	else if (temp_text == 'S') {
		return 1;
	}
	else if (temp_text == 'F') {
		return -1;
	}
	else {
		return 0;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
