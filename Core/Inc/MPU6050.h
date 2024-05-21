#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU6050_ADDR 0xD0 // Addresses of MPU6050, accelerometer and gyroscope registers
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define R_ACCEL 0.01
#define R_GYRO 0.01

void MPU6050_Init(void);
void MPU6050_Read_Accel(float *arr_accel_x, float *arr_accel_y, float *arr_accel_z, float *Kalman_accel_x, float *Kalman_accel_y, float *Kalman_accel_z);
void MPU6050_Read_Gyro(float *arr_gyro_x, float *arr_gyro_y, float *arr_gyro_z, float *Kalman_gyro_x, float *Kalman_gyro_y, float *Kalman_gyro_z);

#endif /* INC_MPU6050_H_ */
