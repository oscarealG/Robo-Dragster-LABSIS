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
#include "avr8-stub.h"
#include "app_api.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

uint16_t gyro_X = 0;
uint16_t gyro_Y = 0;
uint16_t gyro_Z = 0;
uint16_t accel_X = 0;
uint16_t accel_Y = 0;
uint16_t accel_Z = 0;
uint16_t mag_X = 0;
uint16_t mag_Y = 0;
uint16_t mag_Z = 0;

#define BUFFER_SIZE 10 // Tamanho suficiente para armazenar os números convertidos

int main(void)
{
    uart_init(9600); // Inicializa UART com baudrate de 9600
    uart_send_string("uart_init\n");
    I2C_init();
    uart_send_string("I2C_init\n");
    L3GD20_init();
    uart_send_string("L3GD20_init\n");

    getGyroValues(&gyro_X, &gyro_Y, &gyro_Z);
    uart_send_string("getGyroValues\n");
    char buffer[20];
    sprintf(buffer, "Gyro X: %d, Y: %d, Z: %d\r\n", gyro_X, gyro_Y, gyro_Z);
    uart_send_string(buffer);

    LSM303_Accel_Init();
    uart_send_string("LSM303_Accel_Init\n");
    _delay_ms(1000);
    LSM303_Mag_Init();
    uart_send_string("LSM303_Mag_Init\n");
    _delay_ms(1000);
    LSM303_ReadAccel(&accel_X, &accel_Y, &accel_Z);
    sprintf(buffer, "Accel X: %d, Y: %d, Z: %d\r\n", accel_X, accel_Y, accel_Z);
    uart_send_string(buffer);

    // Lê os dados do magnetômetro
    LSM303_ReadMag(&mag_X, &mag_Y, &mag_Z);
    sprintf(buffer, "Mag X: %d, Y: %d, Z: %d\r\n", mag_X, mag_Y, mag_Z);
    uart_send_string(buffer);
    // escrever(valores,11);
    // debug_init();
    // uart_send_string("escrever\n");
    // I2C_init();uart_send_string("I2C_init\n");
    // mpu6050_init();uart_send_string("mpu6050_init\n");
    ssd1306_init();
    uart_send_string("ssd1306_init\n");
    // ssd1306_clear();
    uart_send_string("ssd1306_clear\n");

    // int16_t ax, ay, az;
    // float temperature;
    // char buffer[BUFFER_SIZE];  // Declaração do buffer para armazenar as strings

    // ssd1306_set_cursor(0, 0);        // Posiciona o cursor no topo
    // uart_send_string("Done\n");
    // ssd1306_print("a");   // Exibe o texto
    uart_send_string("Done\n");

    while (1)
    {
        // getGyroValues(&gyro_X, &gyro_Y, &gyro_Z);
        // sprintf(buffer, "%d,%d,%d\r\n", gyro_X, gyro_Y, gyro_Z);
        // uart_send_string(buffer);

        //  LSM303_ReadAccel(&accel_X, &accel_Y, &accel_Z);
        //  sprintf(buffer, "%d,%d,%d\r\n", accel_X, accel_Y, accel_Z);
        // uart_send_string(buffer);

        // LSM303_ReadMag(&mag_X, &mag_Y, &mag_Z);
        // sprintf(buffer, "%d,%d,%d\r\n", mag_X, mag_Y, mag_Z);
        // uart_send_string(buffer);

        // uart_send_string("while\n");
        //  Lê os dados do MPU6050
        // mpu6050_read_accel(&ax, &ay, &az);
        // temperature = mpu6050_read_temperature();

        // Exibe no display
        // ssd1306_clear();
        // ssd1306_print("Accel X: ");
        // ssd1306_print(itoa(ax, buffer, 10));
        // ssd1306_print("\nAccel Y: ");
        // ssd1306_print(itoa(ay, buffer, 10));
        // ssd1306_print("\nAccel Z: ");
        // ssd1306_print(itoa(az, buffer, 10));
        // ssd1306_print("\nTemp: ");
        // ssd1306_print(itoa((int)temperature, buffer, 10));
        // ssd1306_set_cursor(0, 0);        // Posiciona o cursor no topo
        // uart_send_string("Done\n");
        // ssd1306_print("a");   // Exibe o texto
        // uart_send_string("Done\n");
        // uart_send_string("TESTE\n");
        // uart_send_string("Done\n");
        _delay_ms(10); // Delay de 1 segundo
    }
}