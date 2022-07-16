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

uint8_t rx;
uint8_t buf[50], buf_index = 0;
volatile uint8_t flag_uart;

volatile uint8_t flag_set_handle;
volatile uint8_t flag_stroke_handle;
volatile uint8_t flag_peak_handle;
volatile uint8_t flag_auto;
volatile uint8_t count_systick = 0;

volatile uint8_t flag_default_setting = 0;




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
	int num_set_default = 104;
	int num_stroke_default = 116;
	int num_peak_default = 87;

	int num_set = num_set_default;
	int num_stroke = num_stroke_default;
	int num_peak = num_peak_default;




	while (1) {

/*		//do while 처럼 사용하기
		if (flag_default_setting == 1) {
			volatile uint8_t upflag = 0;
			volatile uint8_t downflag = 0;
			printf("start default_setting protocol \r\n");

			while (flag_default_setting == 1) {
				// 같은 버튼으로 여러가지 기능 구현하기 : '1','0'
				if (rx == '1') {
					printf("start updefault (downstroke) setting \r\n");
					upflag = 1;
				}
				else if (rx == '0') {
					printf("start down(default) (upstroke) setting \r\n");
					downflag = 1;
				}

				//scanf 구문이 없으므로 while 문으로 구현하고자 한다.
				//flag_default_setting = 0;
			}
		}*/



		if (count_systick == 1) {
			if (flag_stroke_handle == 1) {
				num_stroke++;
				flag_stroke_handle = 2;
			}
			else if (flag_stroke_handle == 0){
				num_stroke--;
				flag_stroke_handle = 2;
			}
			if (flag_set_handle == 1) {
				num_set++;
				flag_set_handle = 2;
			}
			else if (flag_set_handle == 0){
				num_set--;
				flag_set_handle = 2;
			}
			if (flag_peak_handle == 1) {
				num_peak++;
				flag_peak_handle = 2;
			}
			else if (flag_peak_handle == 0){
				num_peak--;
				flag_peak_handle = 2;
			}
				TIM3->CCR1 = num_peak;
				TIM3->CCR3 = num_set;
				TIM3->CCR4 = num_stroke;

			count_systick = 0;
		}




		if (flag_uart == 1) {

			buf[buf_index] = 0;
			//down stroke
			if (rx == '0') {
				num_set = 105;
				num_peak = 86;
				TIM3->CCR3 = num_set;
				TIM3->CCR1 = num_peak;
				HAL_Delay(10);
				num_stroke = 115;
				TIM3->CCR4 = num_stroke;
				HAL_Delay(450);

				for(int i = 0; i < 27; i++) {
					num_stroke--;
					num_set = 100;
					TIM3->CCR3 = num_set;
					TIM3->CCR4 = num_stroke;
					HAL_Delay(1);
				}
			}
			//up stroke
			else if (rx == '1') {
				num_set = 105;
				TIM3->CCR3 = num_set;
				num_peak = 58;
				TIM3->CCR1 = num_peak;
				HAL_Delay(10);
				num_stroke = 98;
				TIM3->CCR4 = num_stroke;
				HAL_Delay(450);

				for(int i = 0; i < 27; i++) {
					num_stroke++;
					num_set = 99;
					TIM3->CCR3 = num_set;
					TIM3->CCR4 = num_stroke;
					HAL_Delay(1);
				}
			}

			//up down
			else if (rx == '2') {
				num_set = 105;
				TIM3->CCR3 = num_set;
				num_peak = 58;
				TIM3->CCR1 = num_peak;
				HAL_Delay(50);
				num_stroke = 98;
				TIM3->CCR4 = num_stroke;
				HAL_Delay(450);

				for(int i = 0; i < 27; i++) {
					num_stroke++;
					num_set = 98;
					TIM3->CCR3 = num_set;
					TIM3->CCR4 = num_stroke;
					HAL_Delay(1);
				}
				HAL_Delay(100);
				for(int i = 0; i < 27; i++) {
					num_stroke--;
					num_set = 101;
					TIM3->CCR3 = num_set;
					TIM3->CCR4 = num_stroke;
					HAL_Delay(1);
				}
			}

			//down up
			else if (rx == '3') {
				num_set = 105;
				TIM3->CCR3 = num_set;
				num_peak = 58;
				TIM3->CCR1 = num_peak;
				HAL_Delay(50);
				num_stroke = 98;
				TIM3->CCR4 = num_stroke;
				HAL_Delay(450);

				for(int i = 0; i < 27; i++) {
					num_stroke--;
					num_set = 101;
					TIM3->CCR3 = num_set;
					TIM3->CCR4 = num_stroke;
					HAL_Delay(1);
				}
				HAL_Delay(100);
				for(int i = 0; i < 27; i++) {
					num_stroke++;
					num_set = 99;
					TIM3->CCR3 = num_set;
					TIM3->CCR4 = num_stroke;
					HAL_Delay(1);
				}
			}


			memset(buf, 0, sizeof(buf));
			buf_index = 0;
			flag_uart = 0;
		}
			HAL_Delay(1);

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

//		buf[buf_index++] = rx;
		if (rx == '0' || rx == '1'|| rx == '2'|| rx == '3') {
			flag_uart = 1;
		}
		if (rx == 'w' || rx == 'W') {
			flag_set_handle = 1;
		}
		if (rx == 's' || rx == 'S') {
			flag_set_handle = 0;
		}
		if (rx == 'a' || rx == 'A') {
			flag_stroke_handle = 1;
		}
		if (rx == 'd' || rx == 'D') {
			flag_stroke_handle = 0;
		}
		if (rx == 'z' || rx == 'Z') {
			flag_peak_handle = 1;
		}
		if (rx == 'x' || rx == 'X') {
			flag_peak_handle = 0;
		}
		if (rx == 'o') {
			flag_default_setting = 1;
		}
		rx = 0;
		HAL_UART_Receive_IT(&huart3, &rx, 1);
	}
}
void HAL_SYSTICK_Callback(void)
{

	count_systick = 1;
//	printf("%d\r\n", count_systick);

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
