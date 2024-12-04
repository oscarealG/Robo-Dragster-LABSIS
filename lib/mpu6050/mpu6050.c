#include "mpu6050.h"
#include "I2C.h"

void mpu6050_init(void) {
    // Inicia o MPU6050
    I2C_start();
        I2C_write(MPU6050_ADDR<<1); 
    I2C_write(MPU6050_PWR_MGMT_1);  // Registra o controle de alimentação
    I2C_write(0x00);  // Desativa o modo de suspensão
    I2C_stop();
}

void mpu6050_read_accel(int16_t* ax, int16_t* ay, int16_t* az) {
    // Lê os valores de aceleração
    I2C_start();
    I2C_write(MPU6050_ADDR<<1); 
    I2C_write(MPU6050_ACCEL_XOUT_H);
    I2C_stop();

    I2C_start();  // Leitura
    I2C_write(MPU6050_ADDR<<1 | 0x01); 
    *ax = (I2C_read_ack() << 8) | I2C_read_ack();
    *ay = (I2C_read_ack() << 8) | I2C_read_ack();
    *az = (I2C_read_ack() << 8) | I2C_read_nack();
    I2C_stop();
}

void mpu6050_read_gyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    // Lê os valores de giroscópio
    I2C_start();   
     I2C_write(MPU6050_ADDR<<1); 
    I2C_write(MPU6050_GYRO_XOUT_H);
    I2C_stop();

    I2C_start( );  // Leitura
    I2C_write(MPU6050_ADDR<<1| 0x01); 
    *gx = (I2C_read_ack() << 8) | I2C_read_ack();
    *gy = (I2C_read_ack() << 8) | I2C_read_ack();
    *gz = (I2C_read_ack() << 8) | I2C_read_nack();
    I2C_stop();
}

float mpu6050_read_temperature(void) {
    // Lê a temperatura
    int16_t temp_raw;
    I2C_start();
    I2C_write(MPU6050_ADDR<<1); 
    I2C_write(MPU6050_TEMP_OUT_H);
    I2C_stop();

    I2C_start( );  // Leitura
    I2C_write(MPU6050_ADDR<<1| 0x01); 
    temp_raw = (I2C_read_ack() << 8) | I2C_read_nack();
    I2C_stop();
    return (temp_raw / 340.0) + 36.53;  // Temperatura em °C
}
