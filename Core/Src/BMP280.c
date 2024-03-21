#include "BMP280.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

HAL_StatusTypeDef checkP_write;
HAL_StatusTypeDef checkP_read;
HAL_StatusTypeDef checkT_write;
HAL_StatusTypeDef checkT_read;

uint16_t pressure0;

BMP280_S32_t t_fine;
BMP280_S32_t bmp280_compensate_T_int32 (BMP280_S32_t adc_T)
{
	BMP280_S32_t t_fine;
	var1 = ((((adc_T >> 3) - ((BMP280_S32_t) dig_T1 << 1))) * ((BMP280_S32_t) dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((BMP280_S32_t) dig_T1)) * ((adc_T >> 4) - ((BMP280_S32_t) dig_T1))) >> 12) * ((BMP280_S32_t) dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

BMP280_U32_t bmp280_compensate_P_int64 (BMP280_S32_t adc_P)
{
	BMP280_S64_t var1, var2, p;
	var1 = ((BMP280_S64_t)t_fine) - 128000;
	var2 = var1 * var1 * (BMP280_S64_t) dig_P6;
	var2 = var2 + ((var1 * (BMP280_S64_t) dig_P5) << 17);
	var2 = var2 + (((BMP280_S64_t) dig_P4) << 35);
	var1 = ((var1 * var1 * (BMP280_S64_t) dig_P3) >> 8) + ((var1 * (BMP280_S64_t) dig_P2) << 12);
	var1 = (((((BMP280_S64_t) 1) << 47) + var1)) * ((BMP280_S64_t) dig_P1) >> 33;
	if (var1 == 0)
	{
		return 0;
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((BMP280_S64_t) dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((BMP280_S64_t) dig_P8) * p) >> 19;
	p = ((p + var1 +var2) >> 8) + (((BMP280_S64_t) dig_P7) << 4);
	return (BMP280_U32_t) p;
}

void BMP280_Pressure (double *pressure)
{
	uint8_t dataP [3];

//	checkP_write = HAL_I2C_Master_Transmit (&hi2c2, BMP280_ADDRESS << 1, PRESSURE_REG, 1, 1000);
//	if (checkP_write != HAL_OK)
//	{
//		printf ("False");
//	}
//	HAL_Delay (500);
//	checkP_read = HAL_I2C_Master_Receive (&hi2c2, (BMP280_ADDRESS << 1) | 0x01, dataP, 2, 1000);
//	if (checkP_read != HAL_OK)
//	{
//		printf ("False");
//	}
//	HAL_Delay (500);
//
//	pressure0 = ((dataP [0] << 8) | dataP [1]);
//
//	*pressure = pressure0 / 256.0;

	checkP_write = HAL_I2C_Mem_Write (&hi2c2, BMP280_ADDRESS << 1, PRESSURE_REG, I2C_MEMADD_SIZE_8BIT, dataP, 1, 1000);
	if (checkP_write != HAL_OK)
	{
		printf ("False");
	}
	checkP_read = HAL_I2C_Mem_Read (&hi2c2, BMP280_ADDRESS << 1, PRESSURE_REG, I2C_MEMADD_SIZE_8BIT, dataP, 2, 1000);
	if (checkP_read != HAL_OK)
	{
		printf ("False");
	}

	pressure0 = (dataP [0] << 12 | dataP [1] << 4 | dataP [2] >> 4);

	*pressure = pressure0 / 256.0;
}
