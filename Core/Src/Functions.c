#include "Functions.h"
#include "stm32f4xx_hal.h"

float x_accel = 0;
float P_accel = 1;

float x_gyro = 0;
float P_gyro = 1;

void initKalmanFilterAccel(KalmanFilterAccel *filter) // Function to initialize Kalman filter for accelerometer
{
    filter->x = 0;
    filter->P = 1;
}

void initKalmanFilterGyro(KalmanFilterGyro *filter) // Function to initialize Kalman filter for gyroscope
{
    filter->x = 0;
    filter->P = 1;
}

void KalmanFilterUpdateAccel(KalmanFilterAccel *filter, float z, float R, float H) // Function to update Kalman filter for accelerometer
{
    float x_pred = filter->x;
    float P_pred = filter->P;

    float K = P_pred * H / (H * P_pred * H + R);
    filter->x = x_pred + K * (z - H * x_pred);
    filter->P = (1 - K * H) * P_pred;
}

void KalmanFilterUpdateGyro(KalmanFilterGyro *filter, float z, float R, float H) // Function to update Kalman filter for gyroscope
{
    float x_pred = filter->x;
    float P_pred = filter->P;

    float K = P_pred * H / (H * P_pred * H + R);
    filter->x = x_pred + K * (z - H * x_pred);
    filter->P = (1 - K * H) * P_pred;
}
