/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU_HAL_library.h
  * @brief          : Header for IMU_HAL_library.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @description
  *
  * Header of the file with usage of HAL library TM_AHRSIMU methods, which
  * calculates Roll, Pitch and Yaw angles by using AHRS and IMU algorithms.
  * Link: https://stm32f4-discovery.net/hal_api/group___t_m___a_h_r_s_i_m_u.html
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_HAL_LIBRARY_H_
#define IMU_HAL_LIBRARY_H_

/* Includes ------------------------------------------------------------------*/
#include "tm_stm32_ahrs_imu.h"
#include "IMU_algorithms_main.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
/* Gain for calculations and speed to stabilize.
A value less than 0.2 is a good value but it mostly depends on application.*/
#define BETA                0.1f
/* Magnetic inclination for position on earth in units of degrees.
This value depends on GPS coordinates on earth.*/
#define CRACOV_INCLINATION  5.7f  // 2020-11-08

#define GYRO_MAX            2000.0f
#define ACC_MAX             16.0f
#define BITS_MAX            32768.0f

/* External variables --------------------------------------------------------*/
/* Functions prototypes ------------------------------------------------------*/
void IMUHALInit(void);
void HALGyroscopeAccelerometerMagnetometer(void);

void HALprintRoll(void);
void HALprintPitch(void);
void HALprintYaw(void);

#endif /* IMU_HAL_LIBRARY_H_ */

/*****************************END OF FILE*****************************/
