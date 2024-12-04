#include "i2c.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//#define F_CPU 16000000UL  // Clock do microcontrolador
#define SCL_CLOCK 100000L // Clock do barramento I2C

volatile uint8_t i2c_data;		// Variável para armazenar dados recebidos
volatile uint8_t i2c_ready = 0; // Flag para indicar a conclusão da operação

void I2C_init(void)
{
	TWSR = 0x00;		// Prescaler = 1
	TWBR = 0x0C;		// Configuração para SCL = 400kHz
	TWCR = (1 << TWEN); // Habilita TWI
}

void I2C_start(void)
{
	TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)))
		; // Aguarda até que a transmissão seja concluída
}

void I2C_stop(void)
{
	TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
	_delay_ms(1);
}

void I2C_write(uint8_t data)
{
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)))
		;
}

uint8_t I2C_read_ack(void)
{
	TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)))
		;
	return TWDR;
}

uint8_t I2C_read_nack(void)
{
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)))
		;
	return TWDR;
}

uint8_t I2C_ReadRegister(uint8_t deviceAddress, uint8_t Registo)
{
	uint8_t _val;
	I2C_start();
	I2C_write(deviceAddress << 1);
	I2C_write(Registo); // Endereço de início dos dados de aceleração
	I2C_start();
	I2C_write((deviceAddress << 1) | 0x01); // Solicita leitura

	//_val = (I2C_read_ack() << 8) | I2C_read_ack();
	_val = (I2C_read_ack() << 8) | I2C_read_nack();
	I2C_stop();
	return _val;
}

void I2C_WriteRegister(uint8_t deviceAddress, uint8_t Registo, uint8_t val)
{
	I2C_start(); // Inicia a comunicação I2C com o display
	I2C_write(deviceAddress << 1);
	I2C_write(Registo);
	I2C_write(val);
	I2C_stop();
}