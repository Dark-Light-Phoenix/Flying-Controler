#include "BME280.h"
#include "Functions.h"
#include "math.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

void BME280_First_Scan (double *start_pressure, double *start_temperature)
{
	uint32_t pressure0;
	uint8_t PRES_data [3];
	uint8_t TEM_data [3];

	HAL_I2C_Mem_Read (&hi2c2, BME280_ADDRESS << 1, PRESS_MSB_REG, I2C_MEMADD_SIZE_8BIT, PRES_data, 3, 1000); // Reading pressure data from register
	HAL_Delay (1000);
	HAL_I2C_Mem_Read (&hi2c2, BME280_ADDRESS << 1, BME280_TEMPERATURE_MSB_REG, I2C_MEMADD_SIZE_8BIT, TEM_data, 3, 1000); // Reading temperature data from register
	HAL_Delay (1000);

	pressure0 = ((uint32_t) PRES_data [0] << 12 | (uint32_t) PRES_data [1] << 4 | (uint32_t) PRES_data [2] >> 4);

	int32_t adc_T = ((uint32_t) TEM_data [0] << 12) | ((uint32_t) TEM_data [1] << 4) | (TEM_data [2] >> 4);
	int32_t t1, t2, T;

	t1 = ((((adc_T >> 3) - ((int32_t)((uint32_t) TEM_data [0] << 8))) * ((int32_t)*((uint32_t *)0x400001E8))) >> 12);
	t2 = (((((adc_T >> 4) - ((int32_t)((uint32_t) TEM_data [0] << 8))) * ((adc_T >> 4) - ((int32_t)((uint32_t) TEM_data [0] << 8)))) >> 12) * ((int32_t)*((uint32_t *)0x400001EC))) >> 14;

	T = t1 + t2;

	*start_pressure = pressure0 / 256.0;
	*start_temperature = (T / 16384.0);
}

void BME280_ReadPressure (double *pressure)
{
	uint8_t pressure_data [3];

	HAL_I2C_Mem_Read (&hi2c2, BME280_ADDRESS << 1, PRESS_MSB_REG, I2C_MEMADD_SIZE_8BIT, pressure_data, 3, 1000);
	*pressure = ((uint32_t) pressure_data [0] << 12 | (uint32_t) pressure_data [1] << 4 | (uint32_t) pressure_data [2] >> 4);
	*pressure = (*pressure / 256.0); // Convert to Pascal
}

void BME280_ReadTemperature (double *temperature)
{
	uint8_t temperature_data [3];

	HAL_I2C_Mem_Read(&hi2c2, BME280_ADDRESS << 1, BME280_TEMPERATURE_MSB_REG, I2C_MEMADD_SIZE_8BIT, temperature_data, 3, 1000);
	int32_t adc_T = ((uint32_t) temperature_data [0] << 12) | ((uint32_t) temperature_data [1] << 4) | (temperature_data [2] >> 4);
	int32_t t1, t2, T;

	t1 = ((((adc_T >> 3) - ((int32_t)((uint32_t) temperature_data [0] << 8))) * ((int32_t)*((uint32_t *)0x400001E8))) >> 12);
	t2 = (((((adc_T >> 4) - ((int32_t)((uint32_t) temperature_data [0] << 8))) * ((adc_T >> 4) - ((int32_t)((uint32_t) temperature_data [0] << 8)))) >> 12) * ((int32_t)*((uint32_t *)0x400001EC))) >> 14;

	T = t1 + t2;

	*temperature = (T / 16384.0);
}

void BME280_Height(double *start_pressure, double *start_temperature, double *pressure, double *temperature, double *height)
{
	double L = 0.0065; // Temperature gradient
	double exp = 1 / 5.255; // Variable of exponent
	double DeltaP = 1 - (*pressure / *start_pressure); // Variable that means difference between start_pressure and pressure that we take during flight

	*height = (*temperature / L) * pow (DeltaP, exp);
}
