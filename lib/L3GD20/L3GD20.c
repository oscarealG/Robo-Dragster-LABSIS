#include <L3GD20.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include "UART.h"

uint8_t x_lw = 0;
int8_t x_hg = 0;
uint8_t y_lw = 0;
int8_t y_hg = 0;
uint8_t z_lw = 0;
int8_t z_hg = 0;

float dpsComp = 8.75; // Para o caso de 250dps

void L3GD20_init(void)
{
    I2C_WriteRegister(L3GD20_Address, CTRL_REG1, 0b00001111);
    // I2C_WriteRegister(L3GD20_Address, CTRL_REG2, 0b00000000);
    // I2C_WriteRegister(L3GD20_Address, CTRL_REG3, 0b00000000);

    // I2C_WriteRegister(L3GD20_Address, CTRL_REG4, 0b00110000);//scale 2000dps
    I2C_WriteRegister(L3GD20_Address, CTRL_REG4, 0b00000000); // scale 250dps

    // I2C_WriteRegister(L3GD20_Address, CTRL_REG5, 0b00000000);
}


void getGyroValues(int16_t *x, int16_t *y, int16_t *z)
{
    x_lw = I2C_ReadRegister(L3GD20_Address, 0x29);
    x_hg = I2C_ReadRegister(L3GD20_Address, 0x28);

    y_lw = I2C_ReadRegister(L3GD20_Address, 0x2B);
    y_hg = I2C_ReadRegister(L3GD20_Address, 0x2A);

    z_lw = I2C_ReadRegister(L3GD20_Address, 0x2D);
    z_hg = I2C_ReadRegister(L3GD20_Address, 0x2C);

    *x = ((x_hg << 8) | x_lw);
    *y = ((y_hg << 8) | y_lw);
    *z = ((z_hg << 8) | z_lw);
}

float getRealXGyro()
{
    x_lw = I2C_ReadRegister(L3GD20_Address, 0x29);
    x_hg = I2C_ReadRegister(L3GD20_Address, 0x28);
    uint16_t x = ((x_hg << 8) | x_lw);

    return ((float)x * dpsComp) - RateCalibrationX;
}

float getRealYGyro()
{
    y_lw = I2C_ReadRegister(L3GD20_Address, 0x2B);
    y_hg = I2C_ReadRegister(L3GD20_Address, 0x2A);
    uint16_t y = ((y_hg << 8) | y_lw);

    return ((float)y * dpsComp) - RateCalibrationY;
}

float getRealZGyro()
{
    z_lw = I2C_ReadRegister(L3GD20_Address, 0x2D);
    z_hg = I2C_ReadRegister(L3GD20_Address, 0x2C);
    uint16_t z = ((z_hg << 8) | z_lw);

    return ((float)z * dpsComp) - RateCalibrationZ;
}

void CalibGyroValues(int16_t NrOfSamples)
{
    // Retira antiga calibração para não acomular erro
    RateCalibrationX = 0;
    RateCalibrationY = 0;
    RateCalibrationX = 0;

    for (int16_t SampleNr = 0;
         SampleNr < NrOfSamples;
         SampleNr++)
    {
        RateCalibrationX += getRealXGyro();
        RateCalibrationY += getRealYGyro();
        RateCalibrationZ += getRealZGyro();
        _delay_ms(1);
    }

    RateCalibrationX /= NrOfSamples;
    RateCalibrationY /= NrOfSamples;
    RateCalibrationX /= NrOfSamples;
}
