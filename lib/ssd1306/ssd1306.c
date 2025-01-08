#include "ssd1306.h"
#include "I2C.h"
#include "font.h"
#include <util/delay.h>
#include "font.h"

void ssd1306_sendCommand(uint8_t command)
{
    I2C_start();
    I2C_write(SSD1306_I2C_ADDRESS << 1);
    I2C_write(SSD1306_COMMAND);
    I2C_write(command);
    I2C_stop();
}

void ssd1306_sendData(uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        I2C_start();
        I2C_write(SSD1306_I2C_ADDRESS << 1);
        I2C_write(SSD1306_DATA);
        I2C_write(data[i]);
        I2C_stop();
    }
}

// Inicialização do display
void ssd1306_init()
{
    // Sequência de inicialização do SSD1306
    ssd1306_sendCommand(SSD1306_DISPLAY_OFF);     // Desliga o display
    ssd1306_sendCommand(SSD1306_SET_MEMORY_MODE); // Configura o modo de memória
    ssd1306_sendCommand(0x00);                    // Modo horizontal
    ssd1306_sendCommand(SSD1306_DISPLAY_ON);      // Liga o display
}

// Limpa o display
void ssd1306_clearDisplay(void)
{
    memset(pixel_buffer, 0, sizeof(pixel_buffer));
    ssd1306_sendData(pixel_buffer, sizeof(pixel_buffer));
}

// Atualiza o display com os dados do pixel_buffer
void ssd1306_display(void)
{
    // Configurar o display para atualizar
    ssd1306_sendCommand(0x21);              // Set column address
    ssd1306_sendCommand(0);                 // Column start address
    ssd1306_sendCommand(SSD1306_WIDTH - 1); // Column end address

    ssd1306_sendCommand(0x22);                     // Set page address
    ssd1306_sendCommand(0);                        // Page start address
    ssd1306_sendCommand((SSD1306_HEIGHT / 8) - 1); // Page end address

    // Enviar dados do buffer via I2C
    uint16_t count = SSD1306_WIDTH * (SSD1306_HEIGHT / 8);
    uint8_t *ptr = pixel_buffer;

    I2C_start();
    I2C_write(SSD1306_I2C_ADDRESS << 1);
    I2C_write(0x40); // Prefixo de dados

    uint16_t bytesOut = 1; // Já enviamos o prefixo
    while (count--)
    {
        if (bytesOut >= WIRE_MAX)
        {
            I2C_stop();
            I2C_start();
            I2C_write(SSD1306_I2C_ADDRESS << 1);
            I2C_write(0x40); // Prefixo de dados
            bytesOut = 1;
        }
        I2C_write(*ptr++);
        bytesOut++;
    }

    I2C_stop();
}

// Define a posição do cursor
void ssd1306_setCursor(uint8_t x, uint8_t y)
{
    cursor_x = x;
    cursor_y = y;
}

// Escreve texto no pixel_buffer
void ssd1306_writeText(const char *text)
{
    while (*text)
    {
        char c = *text++;
        if (c < 32 || c > 127)
        {
            c = 32; // Substitui caracteres fora do intervalo por espaço
        }

        uint16_t start_pos = cursor_x + (cursor_y * SSD1306_WIDTH);
        if (start_pos + FONT_WIDTH >= sizeof(pixel_buffer))
        {
            break; // Evita escrita fora do buffer
        }

        for (uint8_t i = 0; i < FONT_WIDTH; i++)
        {
            pixel_buffer[start_pos + i] = pgm_read_byte(&font[c - 32][i]);
        }

        cursor_x += FONT_WIDTH + 1; // Espaço entre caracteres
        if (cursor_x + FONT_WIDTH > SSD1306_WIDTH)
        {
            cursor_x = 0;
            cursor_y++;
            if (cursor_y >= SSD1306_HEIGHT / 8)
            {
                cursor_y = 0; // Retorna ao topo
            }
        }
    }
}
