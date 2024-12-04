#ifndef L3GD20_H
#define L3GD20_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <i2c.h>

#define L3GD20_Address 105

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

void L3GD20_init (void);
void getGyroValues(uint16_t* X_Val,uint16_t* Y_Val,uint16_t* Z_Val);

#endif // L3GD20_H