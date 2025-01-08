#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "i2c.h"
#include "LSM303DLH.h"

uint8_t x_l = 0;
int8_t x_h = 0;
uint8_t y_l = 0;
int8_t y_h = 0;
uint8_t z_l = 0;
int8_t z_h = 0;

float GsComp = 4096.0; // Para o caso de _G

void LSM303_Accel_Init(void)
{
    // Configura o registrador CTRL_REG1_A: habilita X, Y, Z e configura frequência de 50 Hz
    // uint8_t data = 0x27; // 0b00100111: Power on, 50Hz, enable XYZ
    I2C_WriteRegister(LSM303_ACCEL_ADDRESS, CTRL_REG1_A, 0x27);
    I2C_WriteRegister(LSM303_ACCEL_ADDRESS, 35, 0x20);
}

void LSM303_Mag_Init(void)
{
    I2C_WriteRegister(LSM303_MAG_ADDRESS, 0x02, 0x00);      // Single-conversion mode
    I2C_WriteRegister(LSM303_MAG_ADDRESS, CRA_REG_M, 0x14); // 30hz output
    I2C_WriteRegister(LSM303_MAG_ADDRESS, 0x01, 0x20);      // gain minimum
    I2C_WriteRegister(LSM303_MAG_ADDRESS, 0x0A, 0x48);      // Datasheet register
    I2C_WriteRegister(LSM303_MAG_ADDRESS, 0x0B, 0x52);      // Datasheet register
    I2C_WriteRegister(LSM303_MAG_ADDRESS, 0x0C, 0x33);      // Datasheet register
}

void LSM303_ReadAccel(int16_t *x, int16_t *y, int16_t *z)
{
    x_l = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_X_L_A);
    x_h = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_X_H_A);

    y_l = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Y_L_A);
    y_h = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Y_H_A);

    z_l = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Z_L_A);
    z_h = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Z_H_A);

    *x = ((x_h << 8) | x_l);
    *y = ((y_h << 8) | y_l);
    *z = ((z_h << 8) | z_l);
}

void LSM303_ReadMag(int16_t *x, int16_t *y, int16_t *z)
{
    x_l = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_X_L_M);
    x_h = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_X_H_M);

    y_l = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_Y_L_M);
    y_h = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_Y_H_M);

    z_l = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_Z_L_M);
    z_h = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_Z_H_M);

    *x = ((x_h << 8) | x_l);
    *y = ((y_h << 8) | y_l);
    *z = ((z_h << 8) | z_l);
}

float getRealXAcc()
{
    x_l = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_X_L_A);
    x_h = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_X_H_A);
    uint16_t x = ((x_h << 8) | x_l);

    return ((float)x / GsComp) - AccCalibX;
}

float getRealYAcc()
{
    y_l = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Y_L_A);
    y_h = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Y_H_A);
    uint16_t y = ((y_h << 8) | y_l);

    return ((float)y / GsComp) - AccCalibY;
}

float getRealZAcc()
{
    z_l = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Z_L_A);
    z_h = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Z_H_A);
    uint16_t z = ((z_h << 8) | z_l);

    return ((float)z / GsComp) - AccCalibZ;
}

float accelRoll()
{
    float real_accel_Y = getRealYAcc();
    float real_accel_X = getRealXAcc();
    float real_accel_Z = getRealZAcc();

    return atan(real_accel_Y / sqrt(real_accel_X * real_accel_X + real_accel_Z * real_accel_Z)) * 1 / (3.14 / 180);
}
float accelPitch()
{
    float real_accel_Y = getRealYAcc();
    float real_accel_X = getRealXAcc();
    float real_accel_Z = getRealZAcc();

    return atan(real_accel_X / sqrt(real_accel_Y * real_accel_Y + real_accel_Z * real_accel_Z)) * 1 / (3.14 / 180);
}

void accelRollandPitch(float *Roll, float *Pitch)
{
    float real_accel_Y = getRealYAcc();
    float real_accel_X = getRealXAcc();
    float real_accel_Z = getRealZAcc();
    
    *Roll = atan(real_accel_Y / sqrt(real_accel_X * real_accel_X + real_accel_Z * real_accel_Z)) * 1 / (3.14 / 180);
    *Pitch = atan(real_accel_X / sqrt(real_accel_Y * real_accel_Y + real_accel_Z * real_accel_Z)) * 1 / (3.14 / 180);
}

void CalibAccValues(int16_t NrOfSamples)
{
    // Retira antiga calibração para não acomular erro
    AccCalibX = 0;
    AccCalibY = 0;
    AccCalibX = 0;

    for (int16_t SampleNr = 0;
         SampleNr < NrOfSamples;
         SampleNr++)
    {
        AccCalibX += getRealYAcc();
        AccCalibY += getRealYAcc();
        AccCalibX += getRealYAcc();
        _delay_ms(1);
    }

    AccCalibX /= NrOfSamples;
    AccCalibY /= NrOfSamples;
    AccCalibX /= NrOfSamples;
}