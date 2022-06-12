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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  NONE  = 0x00U,
  SELECT= 0x01U,
  UP    = 0x02U,
  DOWN  = 0x03U,
  LEFT  = 0X04U,
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

	//LCD variable
	uint8_t row;
	uint8_t mode = 0;


	//ADC variable
	ADC_StatusTypeDef adc_status;
	uint32_t adc_value;

	//RTC variable
	char Time[100];
	char ampm[2][3] = {"AM", "PM"};
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <string.h>

#define LCD_ADDR (0x27 << 1)

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5

void I2C_Scan() {
    char info[] = "Scanning I2C bus...\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);

    HAL_StatusTypeDef res;
    for(uint16_t i = 0; i < 128; i++) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
        if(res == HAL_OK) {
            char msg[64];
            snprintf(msg, sizeof(msg), "0x%02X", i);
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        } else {
            HAL_UART_Transmit(&huart3, (uint8_t*)".", 1, HAL_MAX_DELAY);
        }
    }

    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags) {
    HAL_StatusTypeDef res;
    for(;;) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, lcd_addr, 1, HAL_MAX_DELAY);
        if(res == HAL_OK)
            break;
    }

    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
    data_arr[1] = up|flags|BACKLIGHT;
    data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
    data_arr[3] = lo|flags|BACKLIGHT;

    res = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
//    // data_arr 출력해보기
//    HAL_UART_Transmit(&huart3, data_arr, sizeof(data_arr), 100);
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
    while(*str) {
        LCD_SendData(lcd_addr, (uint8_t)(*str));
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

void loop() {
    HAL_Delay(100);
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

//  HAL_UART_Receive_IT(&huart3, &rx, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	init();
	LCD_Init(LCD_ADDR);
//	LCD_SendCommand(LCD_ADDR, 0b00000001);
	while (1)
  {
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		adc_value = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		sprintf(Time," %s %02x : %02x : %02x", ampm[sTime.TimeFormat], sTime.Hours, sTime.Minutes, sTime.Seconds);

		// LCD 윗줄
		LCD_SendCommand(LCD_ADDR, 0b10000000);
	    LCD_SendString(LCD_ADDR, " Park Jung Hwan");

		// LCD 아랫줄
	    LCD_SendCommand(LCD_ADDR, 0b11000000);
	    LCD_SendString(LCD_ADDR, Time);

	    if (button_status(adc_value) == UP) {
	    	// 커서 blink ON
			LCD_SendCommand(LCD_ADDR, 0b00001111);
			printf("UP\r\n");
	    }
	    if (button_status(adc_value) == DOWN) {
			printf("DOWN\r\n");
	    }
	    if (button_status(adc_value) == LEFT) {
	    	// 커서 왼쪽 이동
			LCD_SendCommand(LCD_ADDR, 0b00010000);
			printf("LEFT\r\n");
	    }
	    if (button_status(adc_value) == RIGHT) {
	    	// 커서 오른쪽 이동
			LCD_SendCommand(LCD_ADDR, 0b00010100);
			printf("RIGHT\r\n");
	    }

	    if (button_status(adc_value) == SELECT) {
	    	LCD_SendString(LCD_ADDR, Time);
	    	mode++;
	    	printf("SELECT\r\n");
	    }

	    while(mode != 0) {

	    	HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 10);
			adc_value = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

	    	if (button_status(adc_value) == UP) {
		    	// 커서 blink
				LCD_SendCommand(LCD_ADDR, 0b00001111);
				printf("UP\r\n");
		    }
		    if (button_status(adc_value) == DOWN) {
		    	mode++;
				printf("DOWN : %d \r\n", mode);
		    }
		    if (button_status(adc_value) == LEFT) {
		    	// 커서 왼쪽이동
		    	LCD_SendCommand(LCD_ADDR, 0b00010000);
				printf("LEFT\r\n");
		    }
		    if (button_status(adc_value) == RIGHT) {
		    	// 커서 오른쪽이동
				LCD_SendCommand(LCD_ADDR, 0b00010100);
				printf("RIGHT\r\n");
		    }

		    if (button_status(adc_value) == SELECT) {
		    	printf("SELECT\r\n");
		        // display & cursor home (keep this!)
		        LCD_SendCommand(LCD_ADDR, 0b00000010);
		    }
			HAL_Delay(1000);
	    }

		memset(buf, 0, sizeof(buf));
		sprintf(buf, "%d\r\n", adc_value);
		HAL_UART_Transmit_IT(&huart3, buf, sizeof(buf));

		HAL_Delay(100);


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
}

/* USER CODE BEGIN 4 */
ADC_StatusTypeDef button_status(uint32_t value) {

	if (value < 100) return UP;
	if (800 < value && value < 900) return DOWN;
	if (1800 < value && value < 2000) return LEFT;
	if (2800 < value && value < 3000) return RIGHT;
	if (4000 < value && value < 5000) return SELECT;

	return NONE;
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
  while (1)
  {
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
