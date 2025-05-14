#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f4xx_hal.h"
#include "stdio.h"

// Định nghĩa các chân kết nối
#define LCD_RS_Pin GPIO_PIN_1
#define LCD_RS_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_2
#define LCD_EN_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_3
#define LCD_D4_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_4
#define LCD_D5_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_5
#define LCD_D6_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_6
#define LCD_D7_Port GPIOB

// Hàm khởi tạo
void LCD_Init(void);

// Hàm hiển thị ký tự
void LCD_SendChar(char c);

// Hàm hiển thị chuỗi
void LCD_SendString(char *str);

// Hàm di chuyển con trỏ đến vị trí cụ thể
void LCD_SetCursor(uint8_t row, uint8_t col);

// Hàm xóa màn hình
void LCD_Clear(void);

// Hàm hiển thị khí gas
void LCD_DisplayGasPPM(uint16_t data, uint8_t row, uint8_t col);

#endif /* INC_LCD_H_ */
