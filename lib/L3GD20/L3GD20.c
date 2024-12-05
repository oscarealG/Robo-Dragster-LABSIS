#include <L3GD20.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "UART.h"

uint8_t xMSB;
uint8_t xLSB;
uint8_t yMSB;
uint8_t yLSB;
uint8_t zMSB;
uint8_t zLSB;

void L3GD20_init(void)
{
    I2C_WriteRegister(L3GD20_Address, CTRL_REG1, 0b00001111);
    I2C_WriteRegister(L3GD20_Address, CTRL_REG2, 0b00000000);
    I2C_WriteRegister(L3GD20_Address, CTRL_REG3, 0b00001000);

    // I2C_WriteRegister(L3GD20_Address, CTRL_REG4, 0b00110000);//scale 2000dps
    I2C_WriteRegister(L3GD20_Address, CTRL_REG4, 0b00000000); // scale 250dps

    I2C_WriteRegister(L3GD20_Address, CTRL_REG5, 0b00000000);
}

void getGyroValues(uint16_t *X_Val, uint16_t *Y_Val, uint16_t *Z_Val)
{
    xMSB = I2C_ReadRegister(L3GD20_Address, 0x29);
    xLSB = I2C_ReadRegister(L3GD20_Address, 0x28);

    yMSB = I2C_ReadRegister(L3GD20_Address, 0x2B);
    yLSB = I2C_ReadRegister(L3GD20_Address, 0x2A);

    zMSB = I2C_ReadRegister(L3GD20_Address, 0x2D);
    zLSB = I2C_ReadRegister(L3GD20_Address, 0x2C);
    
    *X_Val = ((xMSB << 8) | xLSB);
    *Y_Val = ((yMSB << 8) | yLSB);
    *Z_Val = ((zMSB << 8) | zLSB);
}
