/*
 * lcd.c
 *
 *  Created on: Nov 27, 2024
 *      Author: THANH
 */
#include "lcd.h"
#include "stm32f4xx_hal.h"
#include "string.h"



// Hàm tạo độ trễ nhỏ
static void LCD_Delay(uint16_t ms) {
    HAL_Delay(ms);
}

// Gửi tín hiệu đến LCD
static void LCD_Enable(void) {
    HAL_GPIO_WritePin(LCD_EN_Port, LCD_EN_Pin, GPIO_PIN_SET);
    LCD_Delay(1);  // Tạo xung kích
    HAL_GPIO_WritePin(LCD_EN_Port, LCD_EN_Pin, GPIO_PIN_RESET);
    LCD_Delay(1);
}

// Gửi 4 bit dữ liệu
static void LCD_Send4Bits(uint8_t data) {
    HAL_GPIO_WritePin(LCD_D4_Port, LCD_D4_Pin, (data >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_D5_Port, LCD_D5_Pin, (data >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_D6_Port, LCD_D6_Pin, (data >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_D7_Port, LCD_D7_Pin, (data >> 3) & 0x01);
    LCD_Enable();
}

// Gửi lệnh hoặc dữ liệu (rs = 0: lệnh, rs = 1: dữ liệu)
static void LCD_Send(uint8_t data, GPIO_PinState rs) {
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, rs);
    LCD_Send4Bits(data >> 4);  // Gửi 4 bit cao
    LCD_Send4Bits(data & 0x0F); // Gửi 4 bit thấp
}

// Hàm khởi tạo LCD
void LCD_Init(void) {
    // Đặt tất cả các chân làm Output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LCD_RS_Pin | LCD_EN_Pin | LCD_D4_Pin | LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Đợi LCD khởi động
    LCD_Delay(20);

    // Chuyển sang chế độ 4-bit
    LCD_Send4Bits(0x03);
    LCD_Delay(5);
    LCD_Send4Bits(0x03);
    LCD_Delay(1);
    LCD_Send4Bits(0x03);
    LCD_Send4Bits(0x02);

    // Cấu hình LCD
    LCD_Send(0x28, GPIO_PIN_RESET); // Giao diện 4-bit, 2 dòng, font 5x8
    LCD_Send(0x0C, GPIO_PIN_RESET); // Bật màn hình, tắt con trỏ
    LCD_Send(0x06, GPIO_PIN_RESET); // Tự động dịch con trỏ sang phải
    LCD_Clear();
}

// Hàm hiển thị ký tự
void LCD_SendChar(char c) {
    LCD_Send(c, GPIO_PIN_SET);
}

// Hàm hiển thị chuỗi
void LCD_SendString(char *str) {
    while (*str) {
        LCD_SendChar(*str++);
    }
}

// Hàm di chuyển con trỏ
void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    LCD_Send(addr, GPIO_PIN_RESET);
}

// Hàm xóa màn hình
void LCD_Clear(void) {
    LCD_Send(0x01, GPIO_PIN_RESET);
    LCD_Delay(2);
}

// Hàm hiển thị khí gas
void LCD_DisplayGasPPM(uint16_t data, uint8_t row, uint8_t col){
	char buffer[50];
	sprintf(buffer, "GAS: %d ppm", data);
	//LCD_Clear();
	LCD_SetCursor(row, col);
	LCD_SendString(buffer);
}


