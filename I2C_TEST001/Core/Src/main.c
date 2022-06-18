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
#include <string.h>

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

//TIM variable
uint32_t get_time = 0;
uint32_t get_time_apply = 0;
uint32_t apply_flag = 0;

//LCD variable

//i2c scan()
HAL_StatusTypeDef res;
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
// RTC_mode variable
char Time_temp[20];
RTC_TimeTypeDef sTime_temp;
RTC_TimeTypeDef sTime_AL;

// I2C variable
HAL_StatusTypeDef res;

// UART variable
uint32_t buf[20], buf_index = 0;
uint32_t rx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart3, &ch, 1, 100);
	return ch;
}

ADC_StatusTypeDef button_status(uint32_t value);
void I2C_Scan();
HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data,
		uint8_t flags);
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd);
void LCD_SendData(uint8_t lcd_addr, uint8_t data);
void LCD_Init(uint8_t lcd_addr);
void init();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define LCD_ADDR (0x27 << 1)

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5

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
	MX_I2C1_Init();
	MX_USART3_UART_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);

//  HAL_UART_Receive_IT(&huart3, &rx, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	init();
	LCD_Init(LCD_ADDR);
//	LCD_SendCommand(LCD_ADDR, 0b00000001);

	//init the time_temp
	sTime_temp.Hours = 0;
	sTime_temp.Minutes = 0;
	sTime_temp.Seconds = 0;
	sTime_temp.TimeFormat = 0;

	while (1) {
		//Main loop
		while (mode == 0) {
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
			HAL_ADC_Start(&hadc1);

			sprintf(Time, "%s %02x:%02x:%02x", ampm[sTime.TimeFormat],
					sTime.Hours, sTime.Minutes, sTime.Seconds);

			// LCD up
			LCD_SendCommand(LCD_ADDR, 0b10000000);
			LCD_SendString(LCD_ADDR, "Park Jung Hwan");

			// LCD down
			LCD_SendCommand(LCD_ADDR, 0b11000000);
			LCD_SendString(LCD_ADDR, Time);

			//==========================================================================================================
			//mode choose while loop
			while (rising_edge >= 1) {
				cur_tick = HAL_GetTick();
				tick_gap = cur_tick - start_tick;

				if (tick_gap >= 300) {

					if (rising_edge == 1 && falling_edge >= 1) {
						rising_edge = 0;
						falling_edge = 0;
						mode = 1;
						sprintf(Time_temp, "%s %02x:%02x:%02x",
								ampm[sTime_temp.TimeFormat], sTime_temp.Hours,
								sTime_temp.Minutes, sTime_temp.Seconds);
						LCD_SendCommand(LCD_ADDR, 0b11000000);
						LCD_SendString(LCD_ADDR, Time_temp);
						for (int i = 0; i < 11; i++) {
							LCD_SendCommand(LCD_ADDR, 0b00010000);
						}
						LCD_SendCommand(LCD_ADDR, 0b00001111);

						printf("one click==========================\r\n");
					}

					if (rising_edge >= 2 && falling_edge >= 1) {
						rising_edge = 0;
						falling_edge = 0;
						mode = 2;
						printf("two click++++++++++++++++++++++++++\r\n");
					}

					if (tick_gap >= 2000 && falling_edge == 0) {
						rising_edge = 0;
						falling_edge = 0;
						mode = 3;
						printf("long click//////////////////////////\r\n");
					}
				}
			}
		}
		//==========================================================================================================
		//Set Time loop
		while (mode == 1) {

			// start adc for read adc_value
			HAL_ADC_Start(&hadc1);
			// falling 엣지 값이 섞여 들어가지 않도록 초기화 한다.
			falling_edge = 0;

			// IF USER CLICK THE USER BUTTON
			// USER CAN CHOOSE EXIT OR APPLY
			if (rising_edge >= 1) {
				apply_flag = 1;

				// APPLY and exit
				if (falling_edge == 0 && get_time_apply > 19) {

					// sTime is now applied by user
					sTime.Hours = sTime_temp.Hours;
					sTime.Minutes = sTime_temp.Minutes;
					sTime.Seconds = sTime_temp.Seconds;
					sTime.TimeFormat = sTime_temp.TimeFormat;

					// ===========================================init func

					// init the user button
					apply_flag = 0;
					get_time_apply = 0;
					rising_edge = 0;
					falling_edge = 0;

					// turn off the blink
					LCD_SendCommand(LCD_ADDR, 0b00001110);

					// init the temp
					sTime_temp.Hours = 0;
					sTime_temp.Minutes = 0;
					sTime_temp.Seconds = 0;
					sTime_temp.TimeFormat = 0;

					//break the while
					mode = 0;
					printf("MODE APPLY\r\n");
					// ===========================================init func

				}

				// EXIT without apply
				if (falling_edge > 0) {
					// ===========================================init func

					// init the user button
					apply_flag = 0;
					rising_edge = 0;
					falling_edge = 0;

					// turn off the blink
					LCD_SendCommand(LCD_ADDR, 0b00001110);

					// init the temp
					sTime_temp.Hours = 0;
					sTime_temp.Minutes = 0;
					sTime_temp.Seconds = 0;
					sTime_temp.TimeFormat = 0;

					//break the while
					mode = 0;
					printf("MODE APPLY\r\n");
					// ===========================================init func
				}
			}

			if (get_time > 10) {
				if (up > 0) {
					//AM or PM switching
					if (cursor == 0) {
						if (sTime_temp.TimeFormat == 0) {
							sTime_temp.TimeFormat = 1;
							sprintf(Time_temp, "%s %02x:%02x:%02x",
									ampm[sTime_temp.TimeFormat],
									sTime_temp.Hours, sTime_temp.Minutes,
									sTime_temp.Seconds);
							LCD_SendCommand(LCD_ADDR, 0b11000000);
							LCD_SendString(LCD_ADDR, Time_temp);
							for (int i = 0; i < 11; i++) {
								LCD_SendCommand(LCD_ADDR, 0b00010000);
							}

						} else {
							sTime_temp.TimeFormat = 0;
							sprintf(Time_temp, "%s %02x:%02x:%02x",
									ampm[sTime_temp.TimeFormat],
									sTime_temp.Hours, sTime_temp.Minutes,
									sTime_temp.Seconds);
							LCD_SendCommand(LCD_ADDR, 0b11000000);
							LCD_SendString(LCD_ADDR, Time_temp);
							for (int i = 0; i < 11; i++) {
								LCD_SendCommand(LCD_ADDR, 0b00010000);
							}

						}
					}

					// 10H switching
					else if (cursor == 3) {
						if (sTime_temp.Hours < 10) {
//							sTime_temp.Hours += 10;
//							sprintf(Time_temp, "%s %02x:%02x:%02x",
//									ampm[sTime_temp.TimeFormat],
//									sTime_temp.Hours, sTime_temp.Minutes,
//									sTime_temp.Seconds);
//							LCD_SendCommand(LCD_ADDR, 0b11000000);
//							LCD_SendString(LCD_ADDR, Time_temp);
//							for (int i = 0; i < 8; i++) {
//								LCD_SendCommand(LCD_ADDR, 0b00010000);
//							}
//							printf("sTime_temp.Hours : %d\r\n", sTime_temp.Hours);
						}

					}
					// 1H switching
					else if (cursor == 4) {

					}

					// 10M switching
					else if (cursor == 6) {

					}
					// 1M switching
					else if (cursor == 7) {

					}

					// 10S switching
					else if (cursor == 9) {

					}
					// 1S switching
					else if (cursor == 10) {

					}

					// clear the up flag
					up = 0;
				}
				if (down > 0) {
					if (cursor == 0) {
						if (sTime_temp.TimeFormat == 0) {
							sTime_temp.TimeFormat = 1;
							sprintf(Time_temp, "%s %02x:%02x:%02x",
									ampm[sTime_temp.TimeFormat],
									sTime_temp.Hours, sTime_temp.Minutes,
									sTime_temp.Seconds);
							LCD_SendCommand(LCD_ADDR, 0b11000000);
							LCD_SendString(LCD_ADDR, Time_temp);
							for (int i = 0; i < 11; i++) {
								LCD_SendCommand(LCD_ADDR, 0b00010000);
							}

						} else {
							sTime_temp.TimeFormat = 0;
							sprintf(Time_temp, "%s%02x:%02x:%02x",
									ampm[sTime_temp.TimeFormat],
									sTime_temp.Hours, sTime_temp.Minutes,
									sTime_temp.Seconds);
							LCD_SendCommand(LCD_ADDR, 0b11000000);
							LCD_SendString(LCD_ADDR, Time_temp);
							for (int i = 0; i < 11; i++) {
								LCD_SendCommand(LCD_ADDR, 0b00010000);
							}

						}
					}

					// clear the down flag
					down = 0;
				}

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

				// clear the get_time flag (to measure the time)
				get_time = 0;

			}
		}
		//==========================================================================================================
		//AL loop
		while (mode == 2) {

			if (rising_edge >= 1 && falling_edge >= 1) {
				rising_edge = 0;
				falling_edge = 0;
				mode = 0;
				printf("mode reset\r\n");
			}

			// clear the get_time flag (to measure the time)
			get_time = 0;
		}
		//==========================================================================================================
		//Song choice loop
		while (mode == 3) {

			if (rising_edge >= 1 && falling_edge >= 1) {
				rising_edge = 0;
				falling_edge = 0;
				mode = 0;
				printf("mode reset\r\n");
			}

			// clear the get_time flag (to measure the time)
			get_time = 0;
		}
		//==========================================================================================================
		memset(buf, 0, sizeof(buf));
		sprintf(buf, "%d\r\n", ADC_value);
//		HAL_UART_Transmit_IT(&huart3, buf, sizeof(buf));

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
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
	/* EXTI15_10_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	/* TIM3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* USER CODE BEGIN 4 */
ADC_StatusTypeDef button_status(uint32_t value) {

	if (value < 100)
		return UP;
	if (800 < value && value < 900)
		return DOWN;
	if (1800 < value && value < 2000)
		return LEFT;
	if (2800 < value && value < 3000)
		return RIGHT;
	if (4000 < value && value < 5000)
		return SELECT;

	return NONE;
}

void I2C_Scan() {
	char info[] = "Scanning I2C bus...\r\n";
	HAL_UART_Transmit(&huart3, (uint8_t*) info, strlen(info), HAL_MAX_DELAY);

	for (uint16_t i = 0; i < 128; i++) {
		res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
		if (res == HAL_OK) {
			char msg[64];
			snprintf(msg, sizeof(msg), "0x%02X", i);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg),
			HAL_MAX_DELAY);
		} else {
			HAL_UART_Transmit(&huart3, (uint8_t*) ".", 1, HAL_MAX_DELAY);
		}
	}

	HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data,
		uint8_t flags) {
	HAL_StatusTypeDef res;
	for (;;) {
		res = HAL_I2C_IsDeviceReady(&hi2c1, lcd_addr, 1, HAL_MAX_DELAY);
		if (res == HAL_OK)
			break;
	}

	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
	data_arr[0] = up | flags | BACKLIGHT | PIN_EN;
	data_arr[1] = up | flags | BACKLIGHT;
	data_arr[2] = lo | flags | BACKLIGHT | PIN_EN;
	data_arr[3] = lo | flags | BACKLIGHT;

	res = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr, sizeof(data_arr),
	HAL_MAX_DELAY);
	HAL_Delay(LCD_DELAY_MS);
	return res;
}

void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd) {
	LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data) {
	LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void LCD_Init(uint8_t lcd_addr) {
	// 4-bit mode, 2 lines, 5x7 format
	LCD_SendCommand(lcd_addr, 0b00110000);
	// display & cursor home (keep this!)
	LCD_SendCommand(lcd_addr, 0b00000010);
	// display on, right shift, underline off, blink off
	LCD_SendCommand(lcd_addr, 0b00001100);
	// clear display (optional here)
	LCD_SendCommand(lcd_addr, 0b00000001);
}

void LCD_SendString(uint8_t lcd_addr, char *str) {
	while (*str) {
		LCD_SendData(lcd_addr, (uint8_t) (*str));
		str++;
	}
}

void init() {
	I2C_Scan();
	LCD_Init(LCD_ADDR);

	// set address to 0x00
	LCD_SendCommand(LCD_ADDR, 0b10000000);
	LCD_SendString(LCD_ADDR, " Using 1602 LCD");

	// set address to 0x40
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	LCD_SendString(LCD_ADDR, "  over I2C bus");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	// rising edge
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1) {
		rising_edge++;

		printf("rising edge : %d\r\n", rising_edge);
		start_tick = HAL_GetTick();
	}

	// falling edge
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {
		falling_edge++;
		printf("falling edge : %d\r\n", falling_edge);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM3) {

//		HAL_ADC_PollForConversion(&hadc1, 10);
		ADC_value = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		if (button_status(ADC_value) == UP) {
			up++;
			printf("UP : %d\r\n", up);
		}
		if (button_status(ADC_value) == DOWN) {
			down++;
			printf("DOWN : %d\r\n", down);
		}
		if (button_status(ADC_value) == LEFT) {
			left++;
			printf("LEFT : %d\r\n", left);
		}
		if (button_status(ADC_value) == RIGHT) {
			right++;
			printf("RIGHT : %d\r\n", right);
		}

		if (apply_flag > 0) {
			get_time_apply++;
		}
		get_time++;
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
