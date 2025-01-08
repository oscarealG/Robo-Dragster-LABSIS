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


float RateCalibrationX, RateCalibrationY, RateCalibrationZ;

void L3GD20_init(void);

void CalibGyroValues(int16_t NrOfSamples);

void getGyroValues(int16_t *x, int16_t *y, int16_t *z);

float getRealXGyro();
float getRealYGyro();
float getRealZGyro();

#endif // L3GD20_H