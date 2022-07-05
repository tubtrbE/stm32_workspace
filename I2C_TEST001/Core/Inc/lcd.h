#ifndef __LCD_H__
#define __LCD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern HAL_StatusTypeDef res;

/* USER CODE BEGIN Private defines */

#define LCD_ADDR (0x27 << 1)

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
void I2C_Scan();
HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data,
		uint8_t flags);
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd);
void LCD_SendData(uint8_t lcd_addr, uint8_t data);
void LCD_Init(uint8_t lcd_addr);
void init();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __LCD_H__ */
