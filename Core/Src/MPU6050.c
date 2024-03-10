#include "MPU6050.h"
#include "Functions.h"
#include "stm32f4xx_hal.h"

I2C_HandleTypeDef hi2c1;

double Kalman_accel_x[100] = {0}; // Creating massive for data of Kalman filter for accelerometer variables
double Kalman_accel_y[100] = {0};
double Kalman_accel_z[100] = {0};
double Kalman_gyro_x[100] = {0}; // Creating massive for data of Kalman filter for gyroscope variables
double Kalman_gyro_y[100] = {0};
double Kalman_gyro_z[100] = {0};

double arr_accel_x[100] = {0}; // Creating massive for raw data of accelerometer variables
double arr_accel_y[100] = {0};
double arr_accel_z[100] = {0};
double arr_gyro_x[100] = {0}; // Creating massive for raw data of gyroscope variables
double arr_gyro_y[100] = {0};
double arr_gyro_z[100] = {0};

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

void MPU6050_Read_Accel(void)
{
    KalmanFilterAccel accelFilter;
    initKalmanFilterAccel(&accelFilter);

    for (int i = 0; i < 100; i++)
    {
        uint8_t Rec_Data[6];
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

        Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        Ax = Accel_X_RAW / 16384.0;
        Ay = Accel_Y_RAW / 16384.0;
        Az = Accel_Z_RAW / 16384.0;

        arr_accel_x[i] = Ax;
        arr_accel_y[i] = Ay;
        arr_accel_z[i] = Az;

        KalmanFilterUpdateAccel(&accelFilter, Ax, R_ACCEL, 1); // Apply Kalman filter to accelerometer data

        Kalman_accel_x[i] = accelFilter.x;
        Kalman_accel_y[i] = accelFilter.P;
        Kalman_accel_z[i] = Az;
    }
}

void MPU6050_Read_Gyro(void)
{
    KalmanFilterGyro gyroFilter;
    initKalmanFilterGyro(&gyroFilter);

    for (int i = 0; i < 100; i++)
    {
        uint8_t Rec_Data[6];
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
        Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        Gx = Gyro_X_RAW / 131.0;
        Gy = Gyro_Y_RAW / 131.0;
        Gz = Gyro_Z_RAW / 131.0;

        arr_gyro_x[i] = Gx;
        arr_gyro_y[i] = Gy;
        arr_gyro_z[i] = Gz;

        KalmanFilterUpdateGyro(&gyroFilter, Gz, R_GYRO, 1); // Apply Kalman filter to gyroscope data

        Kalman_gyro_x[i] = gyroFilter.x;
        Kalman_gyro_y[i] = gyroFilter.P;
        Kalman_gyro_z[i] = Gz;
    }
}
