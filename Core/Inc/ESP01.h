#ifndef INC_ESP01_H_
#define INC_ESP01_H_

#include "stdint.h"

void USART_Transmit (uint8_t *data, uint16_t length);
void Transmit_Data(float *arr_accel_x, float *arr_accel_y, float *arr_accel_z, float *arr_gyro_x, float *arr_gyro_y, float *arr_gyro_z, float *Kalman_accel_x, float *Kalman_accel_y, float *Kalman_accel_z, float *Kalman_gyro_x, float *Kalman_gyro_y, float *Kalman_gyro_z, float *start_pressure, float *start_temperature, float *StartHeight, float *pressure, float *temperature, float *height, float *DeltaHeight);

#endif /* INC_ESP01_H_ */
