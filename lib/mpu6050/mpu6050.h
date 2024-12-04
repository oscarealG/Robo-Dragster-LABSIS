#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

// Endereço I2C do MPU6050
#define MPU6050_ADDR 0x68

// Registros do MPU6050
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_TEMP_OUT_H 0x41

// Funções
void mpu6050_init(void);
void mpu6050_read_accel(int16_t* ax, int16_t* ay, int16_t* az);
void mpu6050_read_gyro(int16_t* gx, int16_t* gy, int16_t* gz);
float mpu6050_read_temperature(void);

#endif // MPU6050_H
