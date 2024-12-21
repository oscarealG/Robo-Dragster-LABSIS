#ifndef LSM303DLH_H
#define LSM303DLH_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <i2c.h>

// Endereços I2C do LSM303DLH
#define LSM303_ACCEL_ADDRESS  0x19  // Endereço do acelerômetro
#define LSM303_MAG_ADDRESS    0x1E  // Endereço do magnetômetro

// Registradores do LSM303DLH
#define CTRL_REG1_A           0x20  // Configuração do acelerômetro
#define OUT_X_L_A             0x28  // Acelerômetro eixo X (Low byte)
#define OUT_X_H_A             0x29  // Acelerômetro eixo X (High byte)
#define OUT_Y_L_A             0x2A  // Acelerômetro eixo Y (Low byte)
#define OUT_Y_H_A             0x2B  // Acelerômetro eixo Y (High byte)
#define OUT_Z_L_A             0x2C  // Acelerômetro eixo Z (Low byte)
#define OUT_Z_H_A             0x2D  // Acelerômetro eixo Z (High byte)

#define CRA_REG_M             0x00  // Configuração do magnetômetro
#define OUT_X_H_M             0x03  // Magnetômetro eixo X (High byte)
#define OUT_X_L_M             0x04  // Magnetômetro eixo X (Low byte)
#define OUT_Y_H_M             0x07  // Magnetômetro eixo Y (High byte)
#define OUT_Y_L_M             0x08  // Magnetômetro eixo Y (Low byte)
#define OUT_Z_H_M             0x05  // Magnetômetro eixo Z (High byte)
#define OUT_Z_L_M             0x06  // Magnetômetro eixo Z (Low byte)

void LSM303_Accel_Init(void);
void LSM303_Mag_Init(void);
void LSM303_ReadAccel(int16_t* x, int16_t* y, int16_t* z);
void LSM303_ReadMag(uint16_t* x, uint16_t* y, uint16_t* z);

#endif