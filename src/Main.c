#define F_CPU 16000000UL // Frequência do ATmega328p (16 MHz)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "i2c.h"
#include "mpu6050.h"
#include "ssd1306.h"
#include "L3GD20.h"
#include "UART.h"
#include "LSM303DLH.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int16_t gyro_X = 0;
int16_t gyro_Y = 0;
int16_t gyro_Z = 0;
int16_t accel_X = 0;
int16_t accel_Y = 0;
int16_t accel_Z = 0;
int16_t mag_X = 0;
int16_t mag_Y = 0;
int16_t mag_Z = 0;

float real_gyro_X = 0.0;
float real_gyro_Y = 0.0;
float real_gyro_Z = 0.0;
float real_accel_X = 0.0;
float real_accel_Y = 0.0;
float real_accel_Z = 0.0;
float real_mag_X = 0.0;
float real_mag_Y = 0.0;
float real_mag_Z = 0.0;

int16_t real_accel_X_I = 0;
uint16_t real_accel_X_D = 0;
int16_t real_accel_Y_I = 0;
uint16_t real_accel_Y_D = 0;
int16_t real_accel_Z_I = 0;
uint16_t real_accel_Z_D = 0;

int16_t max_accel_X = INT16_MIN, max_accel_Y = INT16_MIN, max_accel_Z = INT16_MIN;
int16_t min_accel_X = INT16_MAX, min_accel_Y = INT16_MAX, min_accel_Z = INT16_MAX;

uint16_t gyroHeading = 0;
uint16_t accelPitch = 0;
uint16_t accelRoll = 0;
float dt = 0.1;
float PI = 3.14;
float teste = 0.0;
char buffer[100];
#define BUFFER_SIZE 10 // Tamanho suficiente para armazenar os números convertidos

void float_2Int(float num, int16_t *parte_inteira, uint16_t *parte_decimal)
{
    float inteiro;
    float decimal;

    decimal = modff(num, &inteiro);                    // modff funciona melhor para floats
    *parte_inteira = (int16_t)inteiro;                 // Parte inteira com o sinal original
    *parte_decimal = (uint16_t)(fabs(decimal) * 1000); // Parte decimal positiva com 3 casas decimais
}
void update_max_min(float x, float y, float z)
{
    if (x > max_accel_X)
        max_accel_X = x;
    if (y > max_accel_Y)
        max_accel_Y = y;
    if (z > max_accel_Z)
        max_accel_Z = z;

    if (x < min_accel_X)
        min_accel_X = x;
    if (y < min_accel_Y)
        min_accel_Y = y;
    if (z < min_accel_Z)
        min_accel_Z = z;
}

int main(void)
{
    uart_init(9600); // Inicializa UART com baudrate de 9600
    I2C_init();      // Inicializa comunicação I2C
    LSM303_Accel_Init(); // Inicializa o acelerômetro

    while (1)
    {
        // Leitura do acelerômetro
        LSM303_ReadAccel(&accel_X, &accel_Y, &accel_Z);

        // Conversão para valores reais (g)
        real_accel_X = (float)accel_X / 4096.0;
        real_accel_Y = (float)accel_Y / 4096.0;
        real_accel_Z = (float)accel_Z / 4096.0;

        // Separar em parte inteira e decimal
        float_2Int(real_accel_X, &real_accel_X_I, &real_accel_X_D);
        float_2Int(real_accel_Y, &real_accel_Y_I, &real_accel_Y_D);
        float_2Int(real_accel_Z, &real_accel_Z_I, &real_accel_Z_D);

        // Imprimir valores no formato correto
        sprintf(buffer, "Accel: %d,%d,%d | Real: %d.%03d,%d.%03d,%d.%03d\n",
                accel_X, accel_Y, accel_Z,
                real_accel_X_I, real_accel_X_D,
                real_accel_Y_I, real_accel_Y_D,
                real_accel_Z_I, real_accel_Z_D);
        uart_send_string(buffer);

        _delay_ms(100); // Delay para estabilização
    }
}