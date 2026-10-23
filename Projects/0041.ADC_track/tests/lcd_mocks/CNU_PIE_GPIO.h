#ifndef LCD_TEST_GPIO_H
#define LCD_TEST_GPIO_H

#include <stdint.h>

#define GPIO_P2 2
#define GPIO_Pin_2 0x04
#define GPIO_Pin_3 0x08
#define GPIO_Pin_4 0x10
#define GPIO_Pin_5 0x20
#define GPIO_Pin_6 0x40
#define GPIO_OUT_PP 1

void GPIO_Init(uint8_t port, uint8_t pin, uint8_t mode);
void GPIO_Write_Bit(uint8_t port, uint8_t pin, uint8_t value);
void Ms_Delay(uint16_t ms);

#endif
