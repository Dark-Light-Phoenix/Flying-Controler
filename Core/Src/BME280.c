#include "BME280.h"
#include "Functions.h"
#include "math.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

double pressure[100] = {0}; // Creating massive for pressure data
double Height[100] = {0}; // Creating massive for height data that converted from pressure

float Pressure;

void BME280_ReadPressure (void) // Function for reading data from BMP280
{
	for (int i = 0; i < 100; i++)
	{
		uint8_t data [3];
		int32_t rawPressure;

		data [0] = 0xF7; // Address register of pressure
		HAL_I2C_Master_Transmit (&hi2c2, BME280_ADDRESS << 1, data, 1, HAL_MAX_DELAY); // Sending command for reading pressure data from BMP280
		HAL_I2C_Master_Receive (&hi2c2, BME280_ADDRESS << 1, data, 3, HAL_MAX_DELAY);

		rawPressure = (data [0] << 12) | (data [1] << 4) | (data [2] >> 4);
		Pressure = (double) rawPressure / 256.0; // Convert in Pascal

		const double T0 = 15; // Temperature at sea level in Celsius (15 °C)
		const double L = 0.0065;  // Standard temperature lapse rate (6.5 °C/km)
		const double P0 = 101325; // Pressure at sea level in Pascal

		double height = (T0 / L) * log (P0 / Pressure);
		pressure[i] = Pressure;
		Height[i] = height;
	}
}

