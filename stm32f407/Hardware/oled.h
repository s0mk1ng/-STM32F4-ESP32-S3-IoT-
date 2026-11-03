#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_WIDTH      128U
#define OLED_HEIGHT     64U
/* OLED分块刷新：每次发送8字节 */
#define OLED_UPDATE_CHUNK_SIZE  8U

#define OLED_I2C_ADDR_3C    (0x3CU << 1)
#define OLED_I2C_ADDR_3D    (0x3DU << 1)

void OLED_Init(void);

void OLED_Clear(void);
void OLED_Fill(uint8_t value);
void OLED_Update(void);
HAL_StatusTypeDef OLED_UpdateChunk(uint8_t page, uint8_t column);

void OLED_Test(void);

/* 字符显示 */
void OLED_ShowChar(uint8_t x, uint8_t y, char chr);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowFloat2(uint8_t x, uint8_t y, float value);
void OLED_ShowInt16(uint8_t x, uint8_t y, int16_t value);

#endif
