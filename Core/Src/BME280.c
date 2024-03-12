#include "BME280.h"
#include "Functions.h"
#include "math.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

double Pressure [100] = {0}; // Creating massive for pressure data
double Height [100] = {0}; // Creating massive for height data that converted from pressure
double Temperature [100] = {0}; // Creating massive for temperature data

uint32_t pressure;

//float Pressure;

//void BME280_ReadPressure (void) // Function for reading data from BMP280
//{
//	for (int i = 0; i < 100; i++)
//	{
//		uint8_t data [3];
//		int32_t rawPressure;
//
//		data [0] = 0xF7; // Address register of pressure
//		HAL_I2C_Master_Transmit (&hi2c2, BME280_ADDRESS << 1, data, 1, HAL_MAX_DELAY); // Sending command for reading pressure data from BMP280
//		HAL_I2C_Master_Receive (&hi2c2, BME280_ADDRESS << 1, data, 3, HAL_MAX_DELAY);
//
//		rawPressure = (data [0] << 12) | (data [1] << 4) | (data [2] >> 4);
//		Pressure = (double) rawPressure / 256.0; // Convert in Pascal
//
//		const double T0 = 15; // Temperature at sea level in Celsius (15 °C)
//		const double L = 0.0065;  // Standard temperature lapse rate (6.5 °C/km)
//		const double P0 = 101325; // Pressure at sea level in Pascal
//
//		double height = (T0 / L) * log (P0 / Pressure);
//		pressure[i] = Pressure;
//		Height[i] = height;
//	}
//}

void BME280_ReadPressure (void)
{
	int i;

	for (i = 0; i < 100; i++)
	{
		uint8_t pressure_data [3];

		HAL_I2C_Mem_Read (&hi2c2, BME280_ADDRESS, PRESS_MSB_REG, 1, pressure_data, 3, 1000);
		pressure = ((uint32_t) pressure_data [0] << 12 | (uint32_t) pressure_data [1] << 4 | (uint32_t) pressure_data [2] >> 4);
		Pressure [i] = pressure / 256.0; // Convert to Pascal

		const double T0 = 288.15; // Temperature at sea level in Kelvin (15 °C)
		const double M = 0.002896;  //
		const double P0 = 101325; // Pressure at sea level in Pascal
		const double R = 8.31; // Universal gas constant
		const double g = 9.8; // Acceleration free fall

		double height = ((T0 * R) / (g * M)) * log (P0 / Pressure [i]);

		Height [i] = height; // Writing data to massive for height
	}
}

void BME280_ReadTemperature (void)
{
	uint8_t temperature_data [3];
	int i;

	for (i = 0; i < 100; i++)
	{
		HAL_I2C_Mem_Read(&hi2c2, BME280_ADDRESS << 1, BME280_TEMPERATURE_MSB_REG, I2C_MEMADD_SIZE_8BIT, temperature_data, 3, 1000);

        int32_t adc_T = ((uint32_t)temperature_data [0] << 12) | ((uint32_t)temperature_data [1] << 4) | (temperature_data [2] >> 4);
        int32_t t1, t2, T;

        t1 = ((((adc_T >> 3) - ((int32_t)((uint32_t)temperature_data [0] << 8))) * ((int32_t)*((uint32_t *)0x400001E8))) >> 12);
        t2 = (((((adc_T >> 4) - ((int32_t)((uint32_t)temperature_data [0] << 8))) * ((adc_T >> 4) - ((int32_t)((uint32_t)temperature_data [0] << 8)))) >> 12) * ((int32_t)*((uint32_t *)0x400001EC))) >> 14;

        T = t1 + t2;

        float temperature = (T / 16384.0);

        Temperature[i] = temperature; //Writing data to massive for temperature
	}
}
