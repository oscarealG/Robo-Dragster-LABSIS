#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void uart_init(unsigned int baud)
{
    unsigned int ubrr = F_CPU / 16 / baud - 1; // Calcula o valor de UBRR para a taxa de baud

    // Configura a taxa de baud
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)(ubrr);

    // Habilita transmissão
    UCSR0B = (1 << TXEN0);

    // Configura o formato do frame (8 bits de dados, 1 bit de stop)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Função para enviar um caractere pela UART
void uart_transmit(unsigned char data)
{
    // Espera até o buffer de transmissão estar vazio
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    // Transmite o dado
    UDR0 = data;
}

// Função para enviar uma string pela UART
void uart_send_string(const char *str)
{
    while (*str)
    {
        uart_transmit(*str);
        str++;
    }
}
