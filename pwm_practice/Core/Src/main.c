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
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {

	UP = 1,
	DOWN = 0,
} _STROKE;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// count 1ms interrupt
volatile uint32_t count_systick = 0;

//=============uart value===========================
uint8_t rx;
uint8_t buf[10], buf_index = 0;
uint8_t buf_temp[10];
//=============uart value===========================

//=============== setting value =====================
volatile uint8_t flag_default_setting;
volatile uint8_t flag_up_apply;
volatile uint8_t flag_down_apply;
volatile uint8_t flag_set_handle;
volatile uint8_t flag_swing_handle;
volatile uint8_t flag_peak_handle;
//=============== setting value =====================

//============== auto_swing value ========================
volatile uint8_t flag_swing_mode;
volatile uint8_t flag_swing_auto;

//============== auto_swing value ========================




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */


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
int main(void)
{
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
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_UART_Receive_IT(&huart3, &rx, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	int num_set_up_yes = 102;
	int num_set_up_no = 104;
	int num_swing_up_no = 116;
	int num_peak_up_no = 87;


	int num_set_down_no = num_set_up_no;
	int num_swing_down = num_swing_up;
	int num_peak_down = num_peak_up;

	int num_set = num_set_up_no;
	int num_swing = num_swing_up;
	int num_peak = num_peak_up;

	TIM3->CCR3 = num_set;
	TIM3->CCR4 = num_swing;
	TIM3->CCR1 = num_peak;

	while (1) {


		// set the default setting
		if (flag_default_setting == 1) {
			printf("\r\n세팅모드에 진입합니다.\r\n");
			flag_default_setting = 2;
		}
		while (flag_default_setting == 2) {

			//no stroke setting
			if (flag_up_apply == 1) {
				num_set_up_no = num_set;
				num_swing_up = num_swing;
				num_peak_up = num_peak;
				printf("\r\n");
				printf("num_set_up_no : %d\r\n", num_set_up_no);
				printf("num_swing_up : %d\r\n", num_swing_up);
				printf("num_peak_up : %d\r\n", num_peak_up);
				flag_up_apply = 0;
			}
			if (flag_down_apply == 1) {
				if (num_set_down_no   == num_set_up_no   &&
					num_swing_down == num_swing_up &&
					num_peak_down  == num_peak_up) {

					num_set_down_no = 104;
					num_swing_down = 116 - 27;
					num_peak_down = 58;

				}
				else {
					num_set_down_no = num_set;
					num_swing_down = num_swing;
					num_peak_down = num_peak;
				}

				printf("\r\n");
				printf("num_set_down : %d\r\n", num_set_down_no);
				printf("num_swing_down : %d\r\n", num_swing_down);
				printf("num_peak_down : %d\r\n", num_peak_down);
				flag_down_apply = 0;
			}

			// yes stroke setting
			if (flag_up_apply == 2) {
				num_set_up_yes = num_set;
				printf("\r\n");
				printf("num_set_up_yes : %d\r\n", num_set_up_yes);
				flag_up_apply = 0;
			}
			if (flag_down_apply == 2) {
				num_set_down_yes = num_set;
				printf("\r\n");
				printf("num_set_down_yes : %d\r\n", num_set_down_yes);
				flag_down_apply = 0;
			}


				// set up & down
				if (flag_set_handle == 1) {
					num_set++;
					flag_set_handle = 0;
					printf("num_set : %d\r\n", num_set);
				} else if (flag_set_handle == 2) {
					num_set--;
					flag_set_handle = 0;
					printf("num_set : %d\r\n", num_set);
				}

				// swing up & down
				if (flag_swing_handle == 1) {
					num_swing++;
					flag_swing_handle = 0;
					printf("num_swing : %d\r\n", num_swing);
				} else if (flag_swing_handle == 2) {
					num_swing--;
					flag_swing_handle = 0;
					printf("num_swing : %d\r\n", num_swing);
				}

				// peak up & down
				if (flag_peak_handle == 1) {
					num_peak++;
					flag_peak_handle = 0;
					printf("num_peak : %d\r\n", num_peak);
				} else if (flag_peak_handle == 2) {
					num_peak--;
					flag_peak_handle = 0;
					printf("num_peak : %d\r\n", num_peak);
				}


			if (count_systick % 100 == 0) {
				TIM3->CCR3 = num_set;
				TIM3->CCR4 = num_swing;
				TIM3->CCR1 = num_peak;
			}

			if (flag_default_setting == 0) {
				printf("\r\n세팅모드를 종료합니다.\r\n");
			}
		}



		if (flag_swing_mode == 1) {
			printf("\r\n스윙모드에 진입합니다.\r\n");
			flag_swing_mode = 2;
		}
		// swing mode
		while (flag_swing_mode == 2) {

			//down swing(up -> down)
			if (flag_swing_auto == 1) {
				TIM3->CCR3 = num_set_up_no;
				TIM3->CCR4 = num_swing_up;
				TIM3->CCR1 = num_peak_up;
				flag_swing_auto = 0;
			}
			//up swing (down -> up)
			else if (flag_swing_auto == 2) {
				TIM3->CCR3 = num_set_down_no;
				TIM3->CCR4 = num_swing_down;
				TIM3->CCR1 = num_peak_down;
				flag_swing_auto = 0;
			}
			if (flag_swing_mode == 0) {
				printf("\r\n스윙모드를 종료합니다.\r\n");
			}
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART3) {
		HAL_UART_Transmit(&huart3, &rx, 1, 100);

		if (rx == '\n') {
			buf[buf_index] = 0;
			if (buf[0] == '`') {
				flag_default_setting = 1;
			}
			if (buf[0] == '~') {
				flag_swing_mode = 1;
			}


			if (flag_default_setting == 2) {
				/*
				volatile uint8_t flag_up_apply;
				volatile uint8_t flag_down_apply;
				volatile uint8_t flag_set_handle;
				volatile uint8_t flag_swing_handle;
				volatile uint8_t flag_peak_handle;
				*/
				if (buf[0] == 'o' || buf[0] == 'O') {
					flag_up_apply = 1;
				}
				if (buf[0] == 'p' || buf[0] == 'P') {
					flag_down_apply = 1;
				}
				if (buf[0] == 'k' || buf[0] == 'K') {
					flag_up_apply = 2;
				}
				if (buf[0] == 'l' || buf[0] == 'l') {
					flag_down_apply = 2;
				}

				if (buf[0] == 'w' || buf[0] == 'W') {
					flag_set_handle = 1;
				}
				if (buf[0] == 's' || buf[0] == 'S') {
					flag_set_handle = 2;
				}

				if (buf[0] == 'a' || buf[0] == 'A') {
					flag_swing_handle = 1;
				}
				if (buf[0] == 'd' || buf[0] == 'D') {
					flag_swing_handle = 2;
				}

				if (buf[0] == 'q' || buf[0] == 'Q') {
					flag_peak_handle = 1;
				}
				if (buf[0] == 'e' || buf[0] == 'E') {
					flag_peak_handle = 2;
				}
				if (buf[0] == 'x' || buf[0] == 'X') {
					flag_default_setting = 0;
				}
			}

			if (flag_swing_mode == 2) {
				if (buf[0] == 'w' || buf[0] == 'W' || buf[0] == '1') {
					flag_swing_auto = 1;
				}
				if (buf[0] == 's' || buf[0] == 'S' || buf[0] == '0') {
					flag_swing_auto = 2;
				}
				if (buf[0] == 'x' || buf[0] == 'X') {
					flag_swing_mode = 0;
				}
			}

			memset(buf, 0, sizeof(buf));
			buf_index = 0;
		}
		else {
			buf[buf_index++] = rx;
		}
		HAL_UART_Receive_IT(&huart3, &rx, 1);
	}
}
void HAL_SYSTICK_Callback(void)
{
	count_systick++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
