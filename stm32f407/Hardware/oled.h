#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_WIDTH      128U
#define OLED_HEIGHT     64U

#define OLED_I2C_ADDR_3C    (0x3CU << 1)
#define OLED_I2C_ADDR_3D    (0x3DU << 1)

void OLED_Init(void);

void OLED_Clear(void);
void OLED_Fill(uint8_t value);
void OLED_Update(void);

void OLED_Test(void);

/* ×Ö·ûÏÔÊ¾ */
void OLED_ShowChar(uint8_t x, uint8_t y, char chr);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowFloat2(uint8_t x, uint8_t y, float value);
void OLED_ShowInt16(uint8_t x, uint8_t y, int16_t value);

#endif
