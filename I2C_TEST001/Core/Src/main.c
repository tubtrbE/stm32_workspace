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
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include "lcd.h"
#include "flash.h"
#include "song.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	NONE = 0x00U,
	SELECT = 0x01U,
	UP = 0x02U,
	DOWN = 0x03U,
	LEFT = 0X04U,
	RIGHT = 0X05U
} ADC_StatusTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t song_flag = 0;

//TIM_PWM variable
uint32_t count_bit = 0;
uint32_t flag_bit_1ms = 0;
uint32_t flag_alarm = 0;
uint32_t count_note = 0;
uint32_t song_time_division = 0;
uint32_t song_choice_flag = 0;

//TIM_BASIC_TIM variable
uint32_t sTimestart = 0;
uint32_t sTimecur = 0;

uint32_t get_time = 0;
uint32_t apply_flag = 0;
uint32_t get_time_apply = 0;
uint32_t exit_flag = 0;
uint32_t get_time_exit = 0;

//LCD variable
uint8_t lcdup[17] = { };

//i2c scan()
uint8_t row;
uint8_t rising_edge = 0;
uint8_t falling_edge = 0;
uint8_t mode = 0;
uint32_t start_tick = 0;
uint32_t cur_tick = 0;
uint32_t tick_gap = 0;
uint8_t pin_status[2] = { 0 };
uint8_t cursor = 0;

//ADC variable
ADC_StatusTypeDef adc_status;
uint32_t ADC_value;
uint8_t up = 0;
uint8_t down = 0;
uint8_t left = 0;
uint8_t right = 0;

//RTC variable
char Time[20];
char ampm[2][3] = { "AM", "PM" };
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

// RTC_mode1 variable
char Time_temp[20];
RTC_TimeTypeDef sTime_temp;
// RTC_mode2 variable
char Time_AL[20] = { };
RTC_TimeTypeDef sTime_AL;

// UART variable
uint8_t buf[20], buf_index = 0;
uint8_t buf_2[20], buf_index_2 = 0;
uint8_t rx_3;
uint8_t rx_2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart3, &ch, 1, 100);
	return ch;
}

void InitFlag(int num);
char* note_address(int song_num, int count_note);
int time_value(int song_num, int count_note);

// i2c adc func
ADC_StatusTypeDef button_status(uint32_t value);
void screen(int cursor, RTC_TimeTypeDef sTime_screen);
// pwm func
void note(char pitch, char octave, char temp, int time, int volume);
uint32_t pitch_change(char pitch_text);
uint32_t octave_change(char octave_text);
uint32_t temp_change(char temp_text);

void adc_up(int up);
void adc_down(int down);
void adc_left(int left);
void adc_right(int right);

void song_choice_func (uint32_t DATA_32);
void mode_choice();
void mode_func_Normal();
void mode_func_SetTime();
void mode_func_SetAlarm();
void mode_func_SetSong();

uint32_t song_Set() ;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_UART_Receive_IT(&huart2, &rx_2, 1);
	HAL_UART_Receive_IT(&huart3, &rx_3, 1);

	song_flag = *((uint32_t*) 0x0800C000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	init();
	LCD_Init(LCD_ADDR);
	up = 0;
	down = 0;
	left = 0;
	right = 0;

	while (1) {
		//init the time_temp
		sTime_temp.Hours = 0;
		sTime_temp.Minutes = 0;
		sTime_temp.Seconds = 0;
		sTime_temp.TimeFormat = 0;

		//Main loop
		mode_func_Normal();
		mode_func_SetTime();
		mode_func_SetAlarm();
		mode_func_SetSong();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
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
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	// rising edge
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1) {
		rising_edge++;

		printf("rising edge : %d\r\n", rising_edge);
		if (rising_edge == 1) {
			start_tick = HAL_GetTick();
		}
	}

	// falling edge
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {
		if (rising_edge == 0) {
			falling_edge = 0;
		} else {
			falling_edge++;
		}
		printf("falling edge : %d\r\n", falling_edge);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM2) {

//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_PollForConversion(&hadc1, 10);
		ADC_value = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		if (button_status(ADC_value) == UP) {
			up++;
//			printf("UP : %d\r\n", up);
		}
		if (button_status(ADC_value) == DOWN) {
			down++;
//			printf("DOWN : %d\r\n", down);
		}
		if (button_status(ADC_value) == LEFT) {
			left++;
//			printf("LEFT : %d\r\n", left);
		}
		if (button_status(ADC_value) == RIGHT) {
			right++;
//			printf("RIGHT : %d\r\n", right);
		}

		if (apply_flag > 0) {
			get_time_apply++;
		}
		if (exit_flag > 0) {
			get_time_exit++;
		}
		get_time++;
//		printf("%d\r\n", ADC_value);
	}

	if (htim->Instance == TIM4) {
		if (flag_alarm > 0) {
			count_bit++;
		} else {
			count_bit = 0;
		}
		flag_bit_1ms = 1;
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		HAL_UART_Transmit(&huart3, &rx_2, 1, 100);
		HAL_UART_Transmit(&huart2, &rx_2, 1, 100);



		HAL_UART_Receive_IT(&huart2, &rx_2, 1);
	}

	if (huart->Instance == USART3) {
		HAL_UART_Transmit(&huart2, &rx_3, 1, 100);
		HAL_UART_Transmit(&huart3, &rx_3, 1, 100);



		HAL_UART_Receive_IT(&huart3, &rx_3, 1);
	}
}



//init user button & LCD
void InitFlag(int num) {

	LCD_Init(LCD_ADDR);
	apply_flag = 0;
	get_time_apply = 0;
	rising_edge = 0;
	falling_edge = 0;
	mode = num;
	strcpy(lcdup, "");
}
ADC_StatusTypeDef button_status(uint32_t value) {

	if (value < 100)
		return UP;
	if (800 < value && value < 900)
		return DOWN;
	if (1800 < value && value < 2000)
		return LEFT;
	if (2800 < value && value < 3200)
		return RIGHT;
	if (4000 < value && value < 5000)
		return SELECT;

	return NONE;
}

void screen(int cursor, RTC_TimeTypeDef sTime_screen) {
	sprintf(Time_temp, "%s %02d:%02d:%02d", ampm[sTime_screen.TimeFormat],
			sTime_screen.Hours, sTime_screen.Minutes, sTime_screen.Seconds);
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	LCD_SendString(LCD_ADDR, Time_temp);
	for (int i = 0; i < 11 - cursor; i++) {
		LCD_SendCommand(LCD_ADDR, 0b00010000);
	}
}

void note(char pitch_text, char octave_text, char temp_text, int time,
		int volume) {

	if (flag_bit_1ms == 1) {
		int pitch = pitch_change(pitch_text);
		int octave = octave_change(octave_text);
		int temp = temp_change(temp_text);

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
		TIM3->CCR3 = pitch / volume;

		flag_bit_1ms = 0;
	}
}
uint32_t pitch_change(char pitch_text) {
	if (pitch_text == 'N') {
		return N;
	} else if (pitch_text == 'C') {
		return C;
	} else if (pitch_text == 'D') {
		return D;
	} else if (pitch_text == 'E') {
		return E;
	} else if (pitch_text == 'F') {
		return F;
	} else if (pitch_text == 'G') {
		return G;
	} else if (pitch_text == 'A') {
		return A;
	} else if (pitch_text == 'B') {
		return B;
	} else {
		return N;
	}
}
uint32_t octave_change(char octave_text) {
	return octave_text - '0';
}
uint32_t temp_change(char temp_text) {
	if (temp_text == 'N') {
		return 0;
	} else if (temp_text == 'S') {
		return 1;
	} else if (temp_text == 'F') {
		return -1;
	} else {
		return 0;
	}
}

char* note_address(int song_num, int count_note) {
	char *song_temp_note;

	if (song_num == 1) {
		song_temp_note = &song_note_1[count_note][0];
	} else if (song_num == 2) {
		song_temp_note = &song_note_2[count_note][0];
	}

	return song_temp_note;
}

int time_value(int song_num, int count_note) {
	int song_time;

	if (song_num == 1) {
		song_time = song_time_1[count_note];
	}

	else if (song_num == 2) {
		song_time = song_time_2[count_note];
	}
	return song_time;
}

void adc_up(int up) {
	if (up > 0) {
		//AM or PM switching
		if (cursor == 0) {
			if (sTime_AL.TimeFormat == 0) {
				sTime_AL.TimeFormat = 1;
				if (sTime_AL.Hours == 0) {
					sTime_AL.Hours = 12;
				}
				screen(cursor, sTime_AL);

			} else if (sTime_AL.TimeFormat == 1) {
				sTime_AL.TimeFormat = 0;
				if (sTime_AL.Hours == 12) {
					sTime_AL.Hours = 0;
				}
				screen(cursor, sTime_AL);
			}
		}

		// 10H switching
		else if (cursor == 3) {

			if (sTime_AL.Hours < 3) {
				sTime_AL.Hours += 10;
			}
			screen(cursor, sTime_AL);
		}

		// 1H switching
		else if (cursor == 4) {

			//AM
			if (sTime_AL.TimeFormat == 0) {
				// 0 ~ 11
				if (0 <= sTime_AL.Hours && sTime_AL.Hours < 11) {
					sTime_AL.Hours++;
				}
			}
			//PM
			else if (sTime_AL.TimeFormat == 1) {

				// 1 ~ 12
				if (1 <= sTime_AL.Hours && sTime_AL.Hours < 12) {
					sTime_AL.Hours++;
				}
			}
			screen(cursor, sTime_AL);
		}

		// 10M switching
		else if (cursor == 6) {
			if (0 <= sTime_AL.Minutes && sTime_AL.Minutes < 50) {
				sTime_AL.Minutes += 10;
			}
			screen(cursor, sTime_AL);
		}
		// 1M switching
		else if (cursor == 7) {
			if (0 <= sTime_AL.Minutes && sTime_AL.Minutes < 59) {
				sTime_AL.Minutes += 1;
			}
			screen(cursor, sTime_AL);
		}

		// 10S switching
		else if (cursor == 9) {
			if (0 <= sTime_AL.Seconds && sTime_AL.Seconds < 50) {
				sTime_AL.Seconds += 10;
			}
			screen(cursor, sTime_AL);
		}
		// 1S switching
		else if (cursor == 10) {
			if (0 <= sTime_AL.Seconds && sTime_AL.Seconds < 59) {
				sTime_AL.Seconds += 1;
			}
			screen(cursor, sTime_AL);
		}

		// clear the up flag
		up = 0;
	}
}

void adc_down(int down) {
	if (down > 0) {

		//AM or PM switching
		if (cursor == 0) {
			if (sTime_AL.TimeFormat == 0) {
				sTime_AL.TimeFormat = 1;
				if (sTime_AL.Hours == 0) {
					sTime_AL.Hours = 12;
				}
				screen(cursor, sTime_AL);
			} else if (sTime_AL.TimeFormat == 1) {
				sTime_AL.TimeFormat = 0;
				if (sTime_AL.Hours == 12) {
					sTime_AL.Hours = 0;
				}
				screen(cursor, sTime_AL);
			}
		}

		// 1H switching
		else if (cursor == 4) {
			if (sTime_AL.Hours > 0) {
				sTime_AL.Hours--;
			}
			screen(cursor, sTime_AL);
		}

		// 10M switching
		else if (cursor == 6) {
			if (0 < sTime_AL.Minutes && sTime_AL.Minutes <= 50) {
				sTime_AL.Minutes -= 10;
			}
			screen(cursor, sTime_AL);
		}
		// 1M switching
		else if (cursor == 7) {
			if (0 < sTime_AL.Minutes && sTime_AL.Minutes <= 59) {
				sTime_AL.Minutes -= 1;
			}
			screen(cursor, sTime_AL);
		}

		// 10S switching
		else if (cursor == 9) {
			if (0 < sTime_AL.Seconds && sTime_AL.Seconds <= 50) {
				sTime_AL.Seconds -= 10;
			}
			screen(cursor, sTime_AL);
		}
		// 1S switching
		else if (cursor == 10) {
			if (0 < sTime_AL.Seconds && sTime_AL.Seconds <= 59) {
				sTime_AL.Seconds -= 1;
			}
			screen(cursor, sTime_AL);
		}

		// clear the down flag
		down = 0;
	}
}

void adc_left(int left) {
	if (left > 0) {
		if (cursor > 0) {
			cursor--;
			LCD_SendCommand(LCD_ADDR, 0b00010000);

			if (cursor == 8) {
				cursor--;
				LCD_SendCommand(LCD_ADDR, 0b00010000);
			}

			if (cursor == 5) {
				cursor--;
				LCD_SendCommand(LCD_ADDR, 0b00010000);
			}

			if (cursor == 2) {
				cursor -= 2;
				LCD_SendCommand(LCD_ADDR, 0b00010000);
				LCD_SendCommand(LCD_ADDR, 0b00010000);
			}
		}

		// clear the left flag
		left = 0;
	}
}

void adc_right(int right) {
	if (right > 0) {

		if (cursor < 10) {
			cursor++;
			LCD_SendCommand(LCD_ADDR, 0b00010100);

			if (cursor == 8) {
				cursor++;
				LCD_SendCommand(LCD_ADDR, 0b00010100);
			}

			if (cursor == 5) {
				cursor++;
				LCD_SendCommand(LCD_ADDR, 0b00010100);
			}

			if (cursor == 1) {
				cursor += 2;
				LCD_SendCommand(LCD_ADDR, 0b00010100);
				LCD_SendCommand(LCD_ADDR, 0b00010100);
			}
		}

		//clear the right flag
		right = 0;
	}
}


void song_choice_func (uint32_t DATA_32) {
	if (DATA_32 == 0x00000001) {
		song_choice_flag = 1;
	}
	else if (DATA_32 == 0x00000002) {
		song_choice_flag = 1;
	}
}

void mode_choice() {
	//mode choose while loop
	while (rising_edge >= 1) {
		cur_tick = HAL_GetTick();
		tick_gap = cur_tick - start_tick;

		//remove bounce effect
		if (tick_gap < 100 && rising_edge > 1) {
			rising_edge = 1;
		}

		if (tick_gap >= 300) {

			if (rising_edge == 1 && falling_edge >= 1) {

				// init the temp
				sTime_temp.Hours = 0;
				sTime_temp.Minutes = 0;
				sTime_temp.Seconds = 0;
				sTime_temp.TimeFormat = 0;
				cursor = 0;

				// LCD up
				LCD_Init(LCD_ADDR);
				LCD_SendCommand(LCD_ADDR, 0b10000000);
				strcpy(lcdup, "Set Time Mode");
				LCD_SendString(LCD_ADDR, lcdup);
				// LCD down
				screen(cursor, sTime_temp);

				LCD_SendCommand(LCD_ADDR, 0b00001111);

				//init the user button
				rising_edge = 0;
				falling_edge = 0;
				mode = 1;
				printf("one click==========================\r\n");
			}

			if (rising_edge >= 2 && falling_edge >= 1) {
				// init the temp
				sTime_AL.Hours = 0;
				sTime_AL.Minutes = 0;
				sTime_AL.Seconds = 0;
				sTime_AL.TimeFormat = 0;
				cursor = 0;

				// LCD up
				LCD_Init(LCD_ADDR);
				//blink on
				LCD_SendCommand(LCD_ADDR, 0b00001111);

				LCD_SendCommand(LCD_ADDR, 0b10000000);
				strcpy(lcdup, "Alarm Mode");
				LCD_SendString(LCD_ADDR, lcdup);
				// LCD down
				screen(cursor, sTime_AL);

				//init the user button
				rising_edge = 0;
				falling_edge = 0;
				mode = 2;

				printf("two click++++++++++++++++++++++++++\r\n");
			}

			if (tick_gap >= 2000 && falling_edge == 0) {

				LCD_Init(LCD_ADDR);

				// set address to 0x00
				LCD_SendCommand(LCD_ADDR, 0b10000000);
				LCD_SendString(LCD_ADDR, "Music Setting");

				// set address to 0x40
				LCD_SendCommand(LCD_ADDR, 0b11000000);
				LCD_SendString(LCD_ADDR, "1.Traffic Light");

				rising_edge = 0;
				falling_edge = 0;
				mode = 3;
				printf("long click//////////////////////////\r\n");
			}
		}
	}
}

void mode_func_Normal() {
	while (mode == 0) {
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		HAL_ADC_Start(&hadc1);

		sprintf(Time, "%s %02d:%02d:%02d", ampm[sTime.TimeFormat], sTime.Hours,
				sTime.Minutes, sTime.Seconds);

		if (strcmp(lcdup, "Park Jung Hwan") != 0) {
			LCD_Init(LCD_ADDR);
			strcpy(lcdup, "Park Jung Hwan");
			// LCD up
			LCD_SendCommand(LCD_ADDR, 0b10000000);
			LCD_SendString(LCD_ADDR, lcdup);

			sprintf(Time, "%s %02d:%02d:%02d", ampm[sTime.TimeFormat],
					sTime.Hours, sTime.Minutes, sTime.Seconds);

			// LCD down
			LCD_SendCommand(LCD_ADDR, 0b11000000);
			LCD_SendString(LCD_ADDR, Time);
		}

		sTimestart = sTimecur;
		sTimecur = sTime.Seconds;

		if (sTimecur != sTimestart) {
			// LCD down
			LCD_SendCommand(LCD_ADDR, 0b11000000);
			LCD_SendString(LCD_ADDR, Time);
		}
		////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////
		if (strcmp(Time, Time_AL) == 0) {
			flag_alarm++;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

		}
		if (flag_alarm > 0) {
			int song_temp_time = time_value(song_flag, count_note);
			char *song_temp_note = note_address(song_flag, count_note);

			if (song_temp_note[0] == '0') {
				TIM3->CCR3 = 0;
				count_note = 0;
				flag_alarm = 0;
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
			}


			song_time_division = 2000 / song_temp_time;

			if (song_time_division >= count_bit) {

				char tempP;
				char tempO;
				char tempT;

				tempP = song_temp_note[0];
				tempO = song_temp_note[1];
				tempT = song_temp_note[2];

				note(tempP, tempO, tempT, 2000 / song_time_division,
						2 + (count_bit));

			}

			if (song_time_division < count_bit) {
				count_note++;
				count_bit = 0;
			}

			////////////////////////////////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		//==========================================================================================================
		mode_choice();
	}
}

void mode_func_SetTime() {
		//==========================================================================================================
		//Set Time loop
		while (mode == 1) {

			// start adc for read adc_value
			HAL_ADC_Start(&hadc1);

			// IF USER CLICK THE USER BUTTON
			// USER CAN CHOOSE EXIT OR APPLY
			if (rising_edge >= 1) {

				// this flag is check the exit or apply
				apply_flag = 1;

				// EXIT without apply
				if (falling_edge > 0) {
					InitFlag(0);
					printf("MODE1 exit\r\n");
				}
				// APPLY and exit
				if (falling_edge == 0 && get_time_apply > 4) {

					// sTime is now applied by user
					sTime.Hours = sTime_temp.Hours;
					sTime.Minutes = sTime_temp.Minutes;
					sTime.Seconds = sTime_temp.Seconds;
					sTime.TimeFormat = sTime_temp.TimeFormat;
					HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

					InitFlag(0);
					printf("MODE1 APPLY\r\n");
				}
			}

			if (get_time > 0) {

				adc_up(up);
				adc_down(down);
				adc_left(left);
				adc_right(right);

				// clear the get_time flag (to measure the time)
				get_time = 0;

			}
		}

		//==========================================================================================================
}
void mode_func_SetAlarm() {
		//AL loop
		while (mode == 2) {
			// start adc for read adc_value
			HAL_ADC_Start(&hadc1);

			// IF USER CLICK THE USER BUTTON
			// USER CAN CHOOSE EXIT OR APPLY
			if (rising_edge >= 1) {

				// Alarm init
				sprintf(Time_AL, "");

				// this flag is check the exit or apply
				apply_flag = 1;

				// EXIT without apply
				if (falling_edge > 0) {
					// ===========================================init func

					InitFlag(0);
					printf("MODE exit\r\n");

					// ===========================================init func
				}
				// APPLY and exit
				if (falling_edge == 0 && get_time_apply > 4) {

					sprintf(Time_AL, "%s %02d:%02d:%02d",
							ampm[sTime_AL.TimeFormat], sTime_AL.Hours,
							sTime_AL.Minutes, sTime_AL.Seconds);

					// ===========================================init func
					InitFlag(0);
					printf("MODE APPLY\r\n");

					// ===========================================init func
				}
			}

			if (get_time > 0) {

				adc_up(up);
				adc_down(down);
				adc_left(left);
				adc_right(right);

				// clear the get_time flag (to measure the time)
				get_time = 0;

			}
		}
		//==========================================================================================================
}
void mode_func_SetSong() {
		//Song choice loop
		while (mode == 3) {

			HAL_ADC_Start(&hadc1);
			uint32_t DATA_32;
			if (get_time > 0) {
//				HAL_ADC_Start(&hadc1);

				if (up > 0) {
					song_flag++;

					if (song_flag <= 0) {
						song_flag = 2;
					}
					else if (song_flag > 2) {
						song_flag = 1;
					}
					DATA_32 = song_Set();
					up = 0;
				}
				if (down > 0) {
					song_flag--;
					if (song_flag <= 0) {
						song_flag = 2;
					}
					else if (song_flag > 2) {
						song_flag = 1;
					}
					DATA_32 = song_Set();
					down = 0;
				}

				// clear the get_time flag (to measure the time)
				get_time = 0;
			}


			// USER CAN CHOOSE EXIT OR APPLY
			if (rising_edge >= 1) {

				// this flag is check the exit or apply
				apply_flag = 1;

				// EXIT without apply
				if (falling_edge > 0) {
					InitFlag(0);
					printf("MODE3 exit\r\n");
				}
				// APPLY and exit
				if (falling_edge == 0 && get_time_apply > 4) {

					LCD_Init(LCD_ADDR);

					// set address to 0x00
					LCD_SendCommand(LCD_ADDR, 0b10000000);
					strcpy(lcdup, "Flash Writing");
					LCD_SendString(LCD_ADDR, lcdup);

					// set address to 0x40
					LCD_SendCommand(LCD_ADDR, 0b11000000);
					LCD_SendString(LCD_ADDR, "Wait for a Sec");

					// Flash Writing Course--------------------------------------------------------------------------------------------------------------------------------------------------
					uint32_t ADDR_FLASH_SECTOR = ADDR_FLASH_SECTOR_3;
					FlashWritingOne(ADDR_FLASH_SECTOR, DATA_32);
					song_flag = *((uint32_t*) 0x0800C000);

					// Flash Writing Course--------------------------------------------------------------------------------------------------------------------------------------------------
					InitFlag(0);
					printf("MODE3 APPLY\r\n");
				}
			}
		}
		//==========================================================================================================
}

uint32_t song_Set() {
	uint32_t DATA_32;

	LCD_Init(LCD_ADDR);

	if (song_flag == 1) {
		DATA_32 = 0x00000001;
		// set address to 0x00
		LCD_SendCommand(LCD_ADDR, 0b10000000);
		LCD_SendString(LCD_ADDR, "Music Setting");

		// set address to 0x40
		LCD_SendCommand(LCD_ADDR, 0b11000000);
		LCD_SendString(LCD_ADDR, song_title_1);
	}
	if (song_flag == 2) {
		DATA_32 = 0x00000002;
		// set address to 0x00
		LCD_SendCommand(LCD_ADDR, 0b10000000);
		LCD_SendString(LCD_ADDR, "Music Setting");

		// set address to 0x40
		LCD_SendCommand(LCD_ADDR, 0b11000000);
		LCD_SendString(LCD_ADDR, song_title_2);
	}

	return DATA_32;
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
