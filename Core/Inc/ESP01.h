#ifndef INC_ESP01_H_
#define INC_ESP01_H_

#include "stdint.h"

#define BUFFER 512

extern float arr_accel_x, arr_accel_y, arr_accel_z, arr_gyro_x, arr_gyro_y, arr_gyro_z, Kalman_accel_x, Kalman_accel_y, Kalman_accel_z, Kalman_gyro_x, Kalman_gyro_y, Kalman_gyro_z, StartHeight, DeltaHeight;

void USART_Transmit (uint8_t *data, uint16_t length);
void Transmit_Data(float *arr_accel_x, float *arr_accel_y, float *arr_accel_z, float *arr_gyro_x, float *arr_gyro_y, float *arr_gyro_z, float *Kalman_accel_x, float *Kalman_accel_y, float *Kalman_accel_z, float *Kalman_gyro_x, float *Kalman_gyro_y, float *Kalman_gyro_z, float *StartHeight, float *DeltaHeight);
void ATCommand (char* cmd);
void WEBServer (void);
void ClientRequest (void);


#endif /* INC_ESP01_H_ */
