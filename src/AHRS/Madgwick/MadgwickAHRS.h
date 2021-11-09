/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU_Madgwick.h
  * @brief          : Header for IMU_Madgwick.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @description
  *
  * Header of the file with implementation of Madgwick Algorithm.
  * Link: https://github.com/PaulStoffregen/MadgwickAHRS
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_MADGWICK_H_
#define IMU_MADGWICK_H_

/* Includes ------------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define BETA          0.1f             // 2 * proportional gain (Kp)

/* External variables --------------------------------------------------------*/
extern volatile float beta;            // 2 * proportional gain (Kp)
extern volatile float q0, q1, q2, q3;  // Quaternions of sensor frame relative to auxiliary frame

/* Functions prototypes ------------------------------------------------------*/
void MadgwickGyroscopeAccelerometerMagnetometer(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float inv_sample_freq, float are_angles_computed);
void MadgwickGyroscopeAccelerometer(float gx, float gy, float gz, float ax, float ay, float az, float inv_sample_freq, float are_angles_computed);

#endif /* IMU_MADGWICK_H_ */

/*****************************END OF FILE*****************************/
