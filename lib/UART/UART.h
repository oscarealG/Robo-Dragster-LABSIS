#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


void uart_init(unsigned int baud);

void uart_transmit(unsigned char data);

void uart_send_string(const char *str);

#endif 