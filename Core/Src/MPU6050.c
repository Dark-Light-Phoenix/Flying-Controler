#include "MPU6050.h"
#include "Functions.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
float Kalman_accel_x, Kalman_accel_y, Kalman_accel_z, Kalman_gyro_x, Kalman_gyro_y, Kalman_gyro_z;
float arr_accel_x, arr_accel_y, arr_accel_z, arr_gyro_x, arr_gyro_y, arr_gyro_z;
float Ax, Ay, Az, Gx, Gy, Gz;

int16_t Accel_X_RAW = 0; // Initialization variables for 3 coordinates X, Y, Z of accelerometer
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0; // Initialization variables for 3 coordinates X, Y, Z of gyroscope
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

void MPU6050_Init(void)
{
    uint8_t check, Data;
    HAL_Delay(1000);
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68)
    {
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
    }
}

void MPU6050_Read_Accel(float *arr_accel_x, float *arr_accel_y, float *arr_accel_z, float *Kalman_accel_x, float *Kalman_accel_y, float *Kalman_accel_z)
{
    KalmanFilterAccel accelFilter;
    initKalmanFilterAccel(&accelFilter);

    for (int i = 0; i < 100; i++)
    {
        uint8_t Rec_Data[6];
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

        Accel_X_RAW = (int16_t)(Rec_Data [0] << 8 | Rec_Data [1]);
        Accel_Y_RAW = (int16_t)(Rec_Data [2] << 8 | Rec_Data [3]);
        Accel_Z_RAW = (int16_t)(Rec_Data [4] << 8 | Rec_Data [5]);
        Ax = Accel_X_RAW / 16384.0;
        Ay = Accel_Y_RAW / 16384.0;
        Az = Accel_Z_RAW / 16384.0;

        *arr_accel_x = Ax;
        *arr_accel_y = Ay;
        *arr_accel_z = Az;

        KalmanFilterUpdateAccel(&accelFilter, Ax, R_ACCEL, 1); // Apply Kalman filter to accelerometer data

        *Kalman_accel_x = accelFilter.x;
        *Kalman_accel_y = accelFilter.P;
        *Kalman_accel_z = Az;
    }
}

void MPU6050_Read_Gyro(float *arr_gyro_x, float *arr_gyro_y, float *arr_gyro_z, float *Kalman_gyro_x, float *Kalman_gyro_y, float *Kalman_gyro_z)
{
    KalmanFilterGyro gyroFilter;
    initKalmanFilterGyro(&gyroFilter);

    for (int i = 0; i < 100; i++)
    {
        uint8_t Rec_Data[6];
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
        Gyro_X_RAW = (int16_t)(Rec_Data [0] << 8 | Rec_Data [1]);
        Gyro_Y_RAW = (int16_t)(Rec_Data [2] << 8 | Rec_Data [3]);
        Gyro_Z_RAW = (int16_t)(Rec_Data [4] << 8 | Rec_Data [5]);
        Gx = Gyro_X_RAW / 131.0;
        Gy = Gyro_Y_RAW / 131.0;
        Gz = Gyro_Z_RAW / 131.0;

        *arr_gyro_x = Gx;
        *arr_gyro_y = Gy;
        *arr_gyro_z = Gz;

        KalmanFilterUpdateGyro(&gyroFilter, Gz, R_GYRO, 1); // Apply Kalman filter to gyroscope data

        *Kalman_gyro_x = gyroFilter.x;
        *Kalman_gyro_y = gyroFilter.P;
        *Kalman_gyro_z = Gz;
    }
}
