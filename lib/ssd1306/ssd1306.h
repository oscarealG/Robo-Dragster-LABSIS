#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

// Definições para o display SSD1306
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SSD1306_ADDR 0x3C

void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_print(const char *str);
void ssd1306_set_cursor(uint8_t column, uint8_t page);

#endif // SSD1306_H
