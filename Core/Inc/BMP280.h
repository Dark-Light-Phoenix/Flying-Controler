#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stm32f4xx_hal.h"

#define BMP280_ADDR 0x76 << 1 // Shifted for 8-bit addressing

#define BMP280_TEMP_MSB_REG 0xFA
#define BMP280_PRESS_MSB_REG 0xF7
#define BMP280_CONFIG_REG 0xF5
#define BMP280_CTRL_MEAS_REG 0xF4

#define k 1.38
#define M 0.04648
#define g 9.81
#define P0 1013

typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} BMP280_CalibData;

void BMP280_ReadCalibrationData(I2C_HandleTypeDef *hi2c2, BMP280_CalibData *calib_data);
void BMP280_Init(void);
float BMP280_ReadTemperature(I2C_HandleTypeDef *hi2c2, BMP280_CalibData *calib_data);
float BMP280_ReadPressure(I2C_HandleTypeDef *hi2c2, BMP280_CalibData *calib_data);
void Setup (float *start_temperature, float *start_pressure, float *StartHeight);
void Delta_Height (float *StartHeight, float *height, float *DeltaHeight, float *pressure, float *temperature);

#endif /* INC_BMP280_H_ */
