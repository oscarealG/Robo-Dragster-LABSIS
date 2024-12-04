#include "ssd1306.h"
#include "I2C.h"
#include "font.h"
#include <util/delay.h>

void ssd1306_init(void)
{
    I2C_start(); // Inicia a comunicação I2C com o display
    I2C_write(SSD1306_ADDR << 1);
    I2C_write(0xAE); // Desliga o display
    I2C_write(0xD5); // Configura o clock do display
    I2C_write(0x80); // Define a frequência do clock
    I2C_write(0xA8); // Define o multiplex ratio
    I2C_write(0x3F); // 1/64 multiplex
    I2C_write(0xD3); // Define o deslocamento de linha
    I2C_write(0x00); // Define o deslocamento de linha
    I2C_write(0x40); // Define a linha de início
    I2C_write(0x8D); // Ativa o driver de corrente (charge pump)
    I2C_write(0x14); // Habilita a fonte de tensão para o driver de corrente
    I2C_write(0x20); // Configura o modo de endereço
    I2C_write(0x00); // Modo de endereçamento horizontal
    I2C_write(0xA1); // Inverte a direção X
    I2C_write(0xC8); // Inverte a direção Y
    I2C_write(0xDA); // Configura os pinos VCOM
    I2C_write(0x12); // Ajuste para o pino VCOM
    I2C_write(0x81); // Configura o contraste do display
    I2C_write(0x7F); // Nível de contraste (0x7F é o valor padrão)
    I2C_write(0xA4); // Desliga o modo de inversão
    I2C_write(0xA6); // Modo normal (não inverso)
    I2C_write(0xAF); // Liga o display
    I2C_stop();      // Finaliza a comunicação I2C
}

void ssd1306_clear(void)
{
    // uart_send_string("ssd1306_clear\n");
    for (uint8_t i = 0; i < 8; i++)
    {
        I2C_start(); // Inicia a comunicação I2C com o display
        I2C_write(SSD1306_ADDR << 1);
        I2C_write(0xB0 + i); // Define a página atual (0xB0 a 0xB7 para as 8 páginas)
        I2C_write(0x00);     // Define a coluna de baixo (0x00-0x0F)
        I2C_write(0x10);     // Define a coluna de cima (0x10-0x1F)
        for (uint8_t j = 0; j < 128; j++)
        {
            I2C_write(0x00); // Envia o byte 0 para limpar o pixel
        }
        I2C_stop(); // Finaliza a comunicação I2C
    }
}

void ssd1306_print_old(const char *str)
{
    while (*str)
    {                // Enquanto houver caracteres na string
        I2C_start(); // Inicia a comunicação I2C
        I2C_write(SSD1306_ADDR << 1);
        I2C_write(0x40); // Indica que são dados, e não comandos
        I2C_write(*str); // Envia o caractere para o display
        str++;           // Avança para o próximo caractere
        I2C_stop();      // Finaliza a comunicação I2C
    }
}

void ssd1306_print(const char *str)
{
    while (*str)
    {                          // Enquanto houver caracteres na string
        uint8_t c = *str - 32; // Converte o caractere para índice na tabela (começa em espaço ' ')
        if (c < 0 || c >= 96)
        {          // Verifica se o caractere está fora do intervalo suportado
            c = 0; // Substitui caracteres inválidos por espaço
        }

        I2C_start(); // Inicia a comunicação I2C
        I2C_write(SSD1306_ADDR<< 1);
        I2C_write(0x40); // Indica que são dados

        for (uint8_t i = 0; i < FONT_WIDTH; i++)
        {
            I2C_write(pgm_read_byte(&font[c][i])); // Envia os dados do caractere como bitmap
        }
        I2C_stop(); // Finaliza a comunicação I2C

        str++; // Avança para o próximo caractere
    }
}

void ssd1306_set_cursor(uint8_t column, uint8_t page)
{
    I2C_start();
    I2C_write(SSD1306_ADDR<< 1);
    I2C_write(0x00);                 // Indica comandos
    I2C_write(0xB0 + page);          // Define a página
    I2C_write((column & 0x0F));      // Define a coluna baixa
    I2C_write(0x10 | (column >> 4)); // Define a coluna alta
    I2C_stop();
}
