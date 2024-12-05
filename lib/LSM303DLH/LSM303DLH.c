#include <stdint.h>
#include <stdio.h>
#include "i2c.h"
#include "LSM303DLH.h"

uint8_t x_l = 0;
uint8_t x_h = 0;
uint8_t y_l = 0;
uint8_t y_h = 0;
uint8_t z_l = 0;
uint8_t z_h = 0;

// Função para inicializar o acelerômetro
void LSM303_Accel_Init(void)
{
    // Configura o registrador CTRL_REG1_A: habilita X, Y, Z e configura frequência de 50 Hz
    // uint8_t data = 0x27; // 0b00100111: Power on, 50Hz, enable XYZ
    I2C_WriteRegister(LSM303_ACCEL_ADDRESS, CTRL_REG1_A, 0x27);
}

// Função para inicializar o magnetômetro
void LSM303_Mag_Init(void)
{
    I2C_WriteRegister(LSM303_MAG_ADDRESS, 0x02, 0x00);// Single-conversion mode
    I2C_WriteRegister(LSM303_MAG_ADDRESS, CRA_REG_M, 0x14); //30hz output
    I2C_WriteRegister(LSM303_MAG_ADDRESS, 0x01, 0x20);//gain minimum
    I2C_WriteRegister(LSM303_MAG_ADDRESS, 0x0A, 0x48);//Datasheet register
    I2C_WriteRegister(LSM303_MAG_ADDRESS, 0x0B, 0x52);//Datasheet register
    I2C_WriteRegister(LSM303_MAG_ADDRESS, 0x0C, 0x33);//Datasheet register       
}

// Função para ler dados do acelerômetro
void LSM303_ReadAccel(uint16_t *x, uint16_t *y, uint16_t *z)
{
    x_l = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_X_L_A);
    x_h = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_X_H_A);
    y_l = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Y_L_A);
    y_h = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Y_H_A);
    z_l = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Z_L_A);
    z_h = I2C_ReadRegister(LSM303_ACCEL_ADDRESS, OUT_Z_H_A);

    // Combina os bytes alto e baixo
    *x = ((x_h << 8) | x_l);
    *y = ((y_h << 8) | y_l);
    *z = ((z_h << 8) | z_l);
}

// Função para ler dados do magnetômetro
void LSM303_ReadMag(uint16_t *x, uint16_t *y, uint16_t *z)
{
    x_l = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_X_L_M);
    x_h = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_X_H_M);
    y_l = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_Y_L_M);
    y_h = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_Y_H_M);
    z_l = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_Z_L_M);
    z_h = I2C_ReadRegister(LSM303_MAG_ADDRESS, OUT_Z_H_M);

    // Combina os bytes alto e baixo
    *x = ((x_h << 8) | x_l);
    *y = ((y_h << 8) | y_l);
    *z = ((z_h << 8) | z_l);
}
