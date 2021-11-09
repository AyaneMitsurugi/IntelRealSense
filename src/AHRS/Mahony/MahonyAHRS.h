/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU_Mahony.h
  * @brief          : Header for IMU_Mahony.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @description
  *
  * Header of the file with implementation of Mahony Algorithm.
  * Link: https://github.com/PaulStoffregen/MahonyAHRS
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_MAHONY_H_
#define IMU_MAHONY_H_

/* Includes ------------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define TWO_KP  (2.0f * 0.5f)  // 2 * proportional gain (Kp)
#define TWO_KI  (2.0f * 0.0f)  // 2 * integral gain (Ki)

/* Functions prototypes ------------------------------------------------------*/
void MahonyPrintfIntegralErrorTerms(void);

void MahonyGyroscopeAccelerometerMagnetometer(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float inv_sample_freq, float are_angles_computed);
void MahonyGyroscopeAccelerometer(float gx, float gy, float gz, float ax, float ay, float az, float inv_sample_freq, float are_angles_computed);

#endif /* IMU_MAHONY_H_ */

/*****************************END OF FILE*****************************/
