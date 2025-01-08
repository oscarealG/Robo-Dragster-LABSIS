#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <string.h>

// Configurações do display SSD1306
#define SSD1306_I2C_ADDRESS 0x3C
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

// Comandos do SSD1306
#define SSD1306_COMMAND 0x00
#define SSD1306_DATA 0x40
#define SSD1306_DISPLAY_ON 0xAF
#define SSD1306_DISPLAY_OFF 0xAE
#define SSD1306_SET_MEMORY_MODE 0x20
#define SSD1306_CLEAR_DISPLAY 0xA4

// Buffer para o display
static uint8_t pixel_buffer[SSD1306_WIDTH * (SSD1306_HEIGHT / 8)];
static uint8_t cursor_x = 0;
static uint8_t cursor_y = 0;
static uint8_t WIRE_MAX = 255;

void ssd1306_sendCommand(uint8_t command);
void ssd1306_sendData(uint8_t *data, size_t length);
void ssd1306_init();
void ssd1306_clearDisplay(void);
void ssd1306_display(void);
void ssd1306_setCursor(uint8_t x, uint8_t y);
void ssd1306_writeText(const char *text);

#endif // SSD1306_H
