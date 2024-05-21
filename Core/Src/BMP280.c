#include "BMP280.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdlib.h"

extern I2C_HandleTypeDef hi2c2;
extern BMP280_CalibData calib_data;
extern float pressure, temperature, height, start_pressure, start_temperature, start_height, delta;

void BMP280_ReadCalibrationData(I2C_HandleTypeDef *hi2c2, BMP280_CalibData *calib_data)
{
    uint8_t calib[24];

    if (HAL_I2C_Mem_Read(hi2c2, BMP280_ADDR, 0x88, I2C_MEMADD_SIZE_8BIT, calib, 24, 1000) == HAL_OK)
    {
        calib_data->dig_T1 = (uint16_t)((calib[1] << 8 ) | calib[0]);
        calib_data->dig_T2 = (int16_t)((calib[3] << 8 ) | calib[2]);
        calib_data->dig_T3 = (int16_t)((calib[5] << 8 ) | calib[4]);
        calib_data->dig_P1 = (uint16_t)((calib[7] << 8 ) | calib[6]);
        calib_data->dig_P2 = (int16_t)((calib[9] << 8 ) | calib[8]);
        calib_data->dig_P3 = (int16_t)((calib[11] << 8 ) | calib[10]);
        calib_data->dig_P4 = (int16_t)((calib[13] << 8 ) | calib[12]);
        calib_data->dig_P5 = (int16_t)((calib[15] << 8 ) | calib[14]);
        calib_data->dig_P6 = (int16_t)((calib[17] << 8 ) | calib[16]);
        calib_data->dig_P7 = (int16_t)((calib[19] << 8 ) | calib[18]);
        calib_data->dig_P8 = (int16_t)((calib[21] << 8 ) | calib[20]);
        calib_data->dig_P9 = (int16_t)((calib[23] << 8 ) | calib[22]);
    } else
    {
        printf ("Error");
    }
}

extern BMP280_CalibData calib_data;

void BMP280_Init(void)
{
    uint8_t config[2];

    // Set the control measurement register
    // Normal mode, Temperature oversampling x 1, Pressure oversampling x 1
    config[0] = BMP280_CTRL_MEAS_REG;
    config[1] = 0x27;
    HAL_I2C_Master_Transmit(&hi2c2, BMP280_ADDR, config, 2, 10);

    // Set the configuration register
    // Standby time 1000 ms, filter off
    config[0] = BMP280_CONFIG_REG;
    config[1] = 0xA0;
    HAL_I2C_Master_Transmit(&hi2c2, BMP280_ADDR, config, 2, 10);
}

int32_t t_fine; // Global variable used in pressure calculation

float BMP280_ReadTemperature(I2C_HandleTypeDef *hi2c2, BMP280_CalibData *calib_data)
{
    uint8_t data[3];
    int32_t raw_temp;

    // Read the raw temperature data
    HAL_I2C_Mem_Read(hi2c2, BMP280_ADDR, BMP280_TEMP_MSB_REG, I2C_MEMADD_SIZE_8BIT, data, 3, 100);
    raw_temp = (data[0] << 16) | (data[1] << 8) | data[2];
    raw_temp >>= 4;

    // Compensate the temperature
    int32_t var1, var2;
    var1 = ((((raw_temp >> 3) - ((int32_t)calib_data->dig_T1 << 1))) * ((int32_t)calib_data->dig_T2)) >> 11;
    var2 = (((((raw_temp >> 4) - ((int32_t)calib_data->dig_T1)) * ((raw_temp >> 4) - ((int32_t)calib_data->dig_T1))) >> 12) *
            ((int32_t)calib_data->dig_T3)) >> 14;
    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128) >> 8;
    return T / 100.0; // Convert to Celsius
}

float BMP280_ReadPressure(I2C_HandleTypeDef *hi2c2, BMP280_CalibData *calib_data)
{
    uint8_t data[3];
    int32_t raw_pres;

    // Read the raw pressure data
    HAL_I2C_Mem_Read(hi2c2, BMP280_ADDR, BMP280_PRESS_MSB_REG, I2C_MEMADD_SIZE_8BIT, data, 3, 100);
    raw_pres = (data[0] << 16) | (data[1] << 8) | data[2];
    raw_pres >>= 4;

    // Compensate the pressure
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data->dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data->dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data->dig_P3) >>  8)+ ((var1 * (int64_t)calib_data->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data->dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - raw_pres;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data->dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data->dig_P7) << 4);
    return (float)p / 25600.0; // Convert to hPa
}

void Setup (float *start_temperature, float *start_pressure, float *StartHeight)
{
	  *StartHeight = (-(M * g) / (k * (*start_temperature + 273))) * log(*start_pressure/P0);
}

void Delta_Height (float *StartHeight, float *height, float *DeltaHeight, float *pressure, float *temperature)
{

	*height = (-(M * g) / (k * (*temperature + 273))) * log(*pressure/P0);
	*DeltaHeight = *StartHeight - *height;
}
