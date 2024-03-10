/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define MPU6050_ADDR 0xD0 // Define addresses of MPU6050
//#define SMPLRT_DIV_REG 0x19
//#define GYRO_CONFIG_REG 0x1B
//#define ACCEL_CONFIG_REG 0x1C
//#define ACCEL_XOUT_H_REG 0x3B
//#define TEMP_OUT_H_REG 0x41
//#define GYRO_XOUT_H_REG 0x43
//#define PWR_MGMT_1_REG 0x6B
//#define WHO_AM_I_REG 0x75
//#define R_ACCEL 0.01
//#define R_GYRO 0.01
//#define BME280_ADDRESS 0x76 // Define address of BMP280
//
//int16_t Accel_X_RAW = 0; // Initialization variables for 3 coordinates X, Y, Z of accelerometer
//int16_t Accel_Y_RAW = 0;
//int16_t Accel_Z_RAW = 0;
//
//int16_t Gyro_X_RAW = 0; // Initialization variables for 3 coordinates X, Y, Z of gyroscope
//int16_t Gyro_Y_RAW = 0;
//int16_t Gyro_Z_RAW = 0;

//double Kalman_accel_x[100] = {0}; // Creating massive for data of Kalman filter for accelerometer variables
//double Kalman_accel_y[100] = {0};
//double Kalman_accel_z[100] = {0};
//double Kalman_gyro_x[100] = {0}; // Creating massive for data of Kalman filter for gyroscope variables
//double Kalman_gyro_y[100] = {0};
//double Kalman_gyro_z[100] = {0};
//
//double arr_accel_x[100] = {0}; // Creating massive for raw data of accelerometer variables
//double arr_accel_y[100] = {0};
//double arr_accel_z[100] = {0};
//double arr_gyro_x[100] = {0}; // Creating massive for raw data of gyroscope variables
//double arr_gyro_y[100] = {0};
//double arr_gyro_z[100] = {0};
//
//double pressure[100] = {0}; // Creating massive for pressure data
//double Height[100] = {0}; // Creating massive for height data that converted from pressure

//float Ax, Ay, Az, Gx, Gy, Gz, Pressure; // Crating variables of accelerometer, gyroscope and BMP280

//float x_accel = 0;
//float P_accel = 1;
//
//float x_gyro = 0;
//float P_gyro = 1;

//typedef struct // Kalman filter structure for accelerometer
//{
//    float x;
//    float P;
//} KalmanFilterAccel;
//
//typedef struct // Kalman filter structure for gyroscope
//{
//    float x;
//    float P;
//} KalmanFilterGyro;


//void initKalmanFilterAccel(KalmanFilterAccel *filter) // Function to initialize Kalman filter for accelerometer
//{
//    filter->x = 0;
//    filter->P = 1;
//}
//
//void initKalmanFilterGyro(KalmanFilterGyro *filter) // Function to initialize Kalman filter for gyroscope
//{
//    filter->x = 0;
//    filter->P = 1;
//}

//void KalmanFilterUpdateAccel(KalmanFilterAccel *filter, float z, float R, float H) // Function to update Kalman filter for accelerometer
//{
//    float x_pred = filter->x;
//    float P_pred = filter->P;
//
//    float K = P_pred * H / (H * P_pred * H + R);
//    filter->x = x_pred + K * (z - H * x_pred);
//    filter->P = (1 - K * H) * P_pred;
//}
//
//void KalmanFilterUpdateGyro(KalmanFilterGyro *filter, float z, float R, float H) // Function to update Kalman filter for gyroscope
//{
//    float x_pred = filter->x;
//    float P_pred = filter->P;
//
//    float K = P_pred * H / (H * P_pred * H + R);
//    filter->x = x_pred + K * (z - H * x_pred);
//    filter->P = (1 - K * H) * P_pred;
//}

//void MPU6050_Init(void)
//{
//    uint8_t check, Data;
//    HAL_Delay(1000);
//    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
//
//    if (check == 0x68)
//    {
//        Data = 0;
//        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
//        Data = 0x07;
//        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
//    }
//}
//
//void MPU6050_Read_Accel(void)
//{
//    KalmanFilterAccel accelFilter;
//    initKalmanFilterAccel(&accelFilter);
//
//    for (int i = 0; i < 100; i++)
//    {
//        uint8_t Rec_Data[6];
//        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
//
//        Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
//        Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
//        Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
//        Ax = Accel_X_RAW / 16384.0;
//        Ay = Accel_Y_RAW / 16384.0;
//        Az = Accel_Z_RAW / 16384.0;
//
//        arr_accel_x[i] = Ax;
//        arr_accel_y[i] = Ay;
//        arr_accel_z[i] = Az;
//
//        KalmanFilterUpdateAccel(&accelFilter, Ax, R_ACCEL, 1); // Apply Kalman filter to accelerometer data
//
//        Kalman_accel_x[i] = accelFilter.x;
//        Kalman_accel_y[i] = accelFilter.P;
//        Kalman_accel_z[i] = Az;
//    }
//}
//
//void MPU6050_Read_Gyro(void)
//{
//    KalmanFilterGyro gyroFilter;
//    initKalmanFilterGyro(&gyroFilter);
//
//    for (int i = 0; i < 100; i++)
//    {
//        uint8_t Rec_Data[6];
//        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
//        Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
//        Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
//        Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
//        Gx = Gyro_X_RAW / 131.0;
//        Gy = Gyro_Y_RAW / 131.0;
//        Gz = Gyro_Z_RAW / 131.0;
//
//        arr_gyro_x[i] = Gx;
//        arr_gyro_y[i] = Gy;
//        arr_gyro_z[i] = Gz;
//
//        KalmanFilterUpdateGyro(&gyroFilter, Gz, R_GYRO, 1); // Apply Kalman filter to gyroscope data
//
//        Kalman_gyro_x[i] = gyroFilter.x;
//        Kalman_gyro_y[i] = gyroFilter.P;
//        Kalman_gyro_z[i] = Gz;
//    }
//}

//void BMP280_ReadPressure (void) // Function for reading data from BMP280
//{
//	for (int i = 0; i < 100; i++)
//	{
//		uint8_t data [3];
//		int32_t rawPressure;
//
//		data [0] = 0xF7; // Address register of pressure
//		HAL_I2C_Master_Transmit (&hi2c2, BMP280_ADDRESS << 1, data, 1, HAL_MAX_DELAY); // Sending command for reading pressure data from BMP280
//		HAL_I2C_Master_Receive (&hi2c2, BMP280_ADDRESS << 1, data, 3, HAL_MAX_DELAY);
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MPU6050_Read_Accel();
	  MPU6050_Read_Gyro();
	  BME280_ReadPressure();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
