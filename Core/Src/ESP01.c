#include "ESP01.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart1;
extern float arr_accel_x, arr_accel_y, arr_accel_z, arr_gyro_x, arr_gyro_y, arr_gyro_z, Kalman_accel_x, Kalman_accel_y, Kalman_accel_z, Kalman_gyro_x, Kalman_gyro_y, Kalman_gyro_z, start_pressure, start_temperature, StartHeight, pressure, temperature, height, DeltaHeight;

void USART_Transmit (uint8_t *data, uint16_t length)
{
	HAL_UART_Transmit (&huart1, data, length, HAL_MAX_DELAY);
}

void Transmit_Data(float *arr_accel_x, float *arr_accel_y, float *arr_accel_z, float *arr_gyro_x, float *arr_gyro_y, float *arr_gyro_z, float *Kalman_accel_x, float *Kalman_accel_y, float *Kalman_accel_z, float *Kalman_gyro_x, float *Kalman_gyro_y, float *Kalman_gyro_z, float *start_pressure, float *start_temperature, float *StartHeight, float *pressure, float *temperature, float *height, float *DeltaHeight)
{
    char buffer[64];

    sprintf(buffer, "X%f", *arr_accel_x);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "Y%f", *arr_accel_y);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "Z%f", *arr_accel_z);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "GX%f", *arr_gyro_x);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "GY%f", *arr_gyro_y);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "GZ%f", *arr_gyro_z);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "KAX%f", *Kalman_accel_x);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "KAY%f", *Kalman_accel_y);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "KAZ%f", *Kalman_accel_z);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "KGX%f", *Kalman_gyro_x);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "KGY%f", *Kalman_gyro_y);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "KGZ%f", *Kalman_gyro_z);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "SP%f", *start_pressure);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "ST%f", *start_temperature);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "SH%f", *StartHeight);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "P%f", *pressure);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "T%f", *temperature);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "H%f", *height);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));

    sprintf(buffer, "DH%f", *DeltaHeight);
    USART_Transmit((uint8_t *)buffer, strlen(buffer));
}
