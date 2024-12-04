#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Frequência do clock do barramento I2C (100kHz por padrão)
#define SCL_CLOCK 100000L  

void I2C_init(void);
void I2C_start(void);
void I2C_write(uint8_t data);
uint8_t I2C_read_ack(void);
uint8_t I2C_read_nack(void);
void I2C_stop(void);
void I2C_WriteRegister(uint8_t deviceAddress,uint8_t Registo,uint8_t val);
uint8_t I2C_ReadRegister(uint8_t deviceAddress,uint8_t Registo);



#endif // I2C_H