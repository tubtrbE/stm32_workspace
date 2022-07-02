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



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void note(char pitch, char octave, char temp, int time, int volume);
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
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);


	char *verse1[] = {

	///////////////////////////////////////////////////////////
			// C도레미 도시
			"C4N", "C5N", "D5N", "E5N", "C5N", "G4N",
			//도시 도레미
			"C5N", "G4N", "C5N", "D5N", "E5N",

			// 도레미 도시
			"F3N", "C5N", "D5N", "E5N", "C5N", "G4N",
			//도시도미 파미레도
			"C5N", "G4N","C5N","E5N","F5N","E5N","D5N","C5N",

			// 도레미 도시
			"C4N", "C5N", "D5N", "E5N", "C5N", "G4N",
			//도시 도레미
			"C5N", "G4N", "C5N", "D5N", "E5N",

			//N미파미파미도 레N
			"N5N","E5N","F5N","E5N","F5N","E5N","C5N"   ,"D5N","N5N",

			// N도레미 도시
			"N5N", "C5N", "D5N", "E5N", "C5N", "G4N",
			//도시 도레미
			"C5N", "G4N", "C5N", "D5N", "E5N",

			// 도레미 도시
			"N5N", "C5N", "D5N", "E5N", "C5N", "G4N",
			//도시도미 파미레도
			"C5N", "G4N","C5N","E5N","F5N","E5N","D5N","C5N",

			// N도레미 도시
			"N5N", "C5N", "D5N", "E5N", "C5N", "G4N",
			//도시 도파미
			"C5N", "G4N", "C5N", "F5N", "E5N",

			//N미파미파미도 레N
			"N5N","E5N","F5N","E5N","F5N","E5N","C5N"   ,"D5N","N5N",

			//N레레레레도시도시도
			"N5N","D5N","D5N","D5N","D5N", "C5N", "G4N", "C5N", "G4N", "C5N",
			//N도미파미레도레
			"N5N","C5N","E5N","F5N","E5N","D5N","C5N","D5N",

			//NN도레미
			"N5N","N5N","C5N","D5N","E5N",

			//N레레레레도시도시도레미
			"N5N","D5N","D5N","D5N","D5N", "C5N", "G4N", "C5N", "G4N", "C5N", "D5N", "E5N",

			//N라라라라솔미솔
			"N4N","A4N","A4N","A4N","A4N","G4N","E4N","G4N",

			//N미레  도라라솔
			"N5N","E5N","D5N",    "C5N","A5N","A5N","G5N",

			//미레도도도레미
			"E5N","D5N","C5N","C5N","C5N","D5N","E5N",

			//미레도파파미도
			"E5N","D5N","C5N","F5N","F5N","E5N","C5N",

			//N도도미파레
			"N5N","C5N","C5N","E5N","F5N","D5N",
/////////////////////////////////////////////////////////////////
			//미레  도라라솔
			"E5N","D5N",    "C5N","A5N","A5N","G5N",
			//미레도도도레미
			"E5N","D5N","C5N","C5N","C5N","D5N","E5N",
			//미레도파파미도 도미레
			"E5N","D5N","C5N","F5N","F5N","E5N","C5N","C5N","E5N","D5N",
			//미파미레도
			"E5N","F5N","E5N","D5N","C5N",


			//노래끝
			"0",
	///////////////////////////////////////////////////////////
			};

	int verse1_time[] = {
			//도레미도시도시도레미
			4,8,8,4,8,8,
			8,8,8,8,2,

			4,8,8,4,8,8,
			8,8,8,8,8,8,8,8,

			4,8,8,4,8,8,
			8,8,8,8,2,

			4,8,8,8,8,8,8,2,2,

			4,8,8,4,8,8,
			8,8,8,8,2,

			4,8,8,4,8,8,
			8,8,8,8,8,8,8,8,

			4,8,8,4,8,8,
			8,8,8,8,2,
			//미파미파미도레
			4,8,8,8,8,8,8,2,2,

			4,8,8,8,8,8,8,8,8,4,
			4,8,8,4,4,8,8,4,

			2,8,8,8,8,
			4,8,8,8,8,8,8,8,8,8,8,2,

			8,8,8,8,8,8,8,8,
			//미레도 라라솔
			1.5,8,8,4,8,8,4,

			//미레도도도레미
			8,8,8,8,8,8,4,

			//미레도 파파미도
			8,8,4,8,8,8,8,

			//N도도미파레
			8,8,4,8,8,4,
///////////////////////////////////////////////////
			//미레도 라라솔
			8,8,4,8,8,4,

			//미레도도도레미
			8,8,8,8,8,8,4,

			//미레도 파파미도  도미레
			8,8,4,8,8,8,8,8,8,2,

			//미파미레도
			8,8,8,8,1
	};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {



//		int i = 0;
//		int j = 0;
//		int count_time = 0;
//		for (int k = 0; k < 30; k++) {
//			uint32_t pitch = 0;
//			uint32_t octave = 0;
//			int time = 0;
//			char tempP = verse1[i][0];
//			char tempO = verse1[i][1];
//			char tempT = verse1[i][2];
//			if (verse1[i] != 0) {
//				time = verse1_time[count_time];
//				count_time++;
//				note(tempP, tempO, tempT, 2000/time, 2);
//				i++;
//			}
//			else {
//				break;
//			}
//		}


		int i = 0;
		int j = 0;
		int count_time = 0;
		while (strlen(verse1[i]) == 3) {
			int time = 0;
			char tempP = verse1[i][0];
			char tempO = verse1[i][1];
			char tempT = verse1[i][2];
				time = verse1_time[count_time];
				count_time++;
				note(tempP, tempO, tempT, 2000 / time, 4);
				i++;
				if (strlen(verse1[i]) == 1){
				break;
			}
		}



//	  note('C', '4', 'N', 500, 2); //?��
//	  note('D', '4', 'N', 500, 4); //?��
//	  note('E', '4', 'N', 500, 4); //�?
//	  note('F', '4', 'N', 500, 4); //?��
//	  note('G', '4', 'N', 500, 4); //?��
//	  note('A', '4', 'N', 500, 4); //?��
//	  note('B', '4', 'N', 500, 4); //?��
//	  note('C', '5', 'N', 1000, 4); //?��

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
void note(char pitch_text, char octave_text, char temp_text, int time, int volume) {


	int pitch = pitch_change(pitch_text);
	int octave = octave_change(octave_text);
	int temp = temp_change(temp_text);

//	printf("%d %d %d %d %d\n\r", pitch, octave, temp, time, volume);


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
		// if printf is not activated this while loop doesnt work correctly
		printf("%d %d %d %d %d\n\r", pitch, octave, temp, time, volume);
//		printf("tick_gap : %d\r\n", tick_gap);
//		printf("\r\n");
		if (tick_gap % 5 == 0 || tick_gap % 4 == 0 || tick_gap % 3 == 0) {
			volume += 2;
		}
	}
}
uint32_t pitch_change (char pitch_text) {
	if (pitch_text == 'N') {
		return N;
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
		return N;
	}
}
uint32_t octave_change (char octave_text) {
	return octave_text - '0';
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
