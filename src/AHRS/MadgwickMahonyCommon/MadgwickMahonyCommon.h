/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU_algorithms_main.h
  * @brief          : Header for IMU_algorithms_main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @description
  *
  * Parameter "IMU_algorithm" decides, which algorithm is being used:
  *
  * 1 => HAL library TM_AHRSIMU
  * Link: https://stm32f4-discovery.net/hal_api/group___t_m___a_h_r_s_i_m_u.html
  *
  * 2 => Mahony Algorithm
  * Link: https://github.com/PaulStoffregen/MahonyAHRS
  *
  * 3 => Madgwick Algorithm
  * Link: https://github.com/PaulStoffregen/MadgwickAHRS
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_ALGORITHMS_MAIN_H_
#define IMU_ALGORITHMS_MAIN_H_

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "../Madgwick/MadgwickAHRS.h"
#include "../Mahony/MahonyAHRS.h"

/* Functions prototypes ------------------------------------------------------*/
void use_specific_IMU_algorithm(uint8_t IMU_algorithm);

float gyroscopeMeasurements();
float accelerometerMeasurements();
float magnetometerMeasurements();

void printGyroscopeMeasurements(void);
void printfAccelerometerMeasurements(void);
void printfMagnetometerMeasurements(void);

float gtRoll();
float getPitch();
float getYaw();

float getRollRadians();
float getPitchRadians();
float getYawRadians();

void printfRoll(void);
void printfPitch(void);
void printfYaw(void);

void printfQuaternions(void);
void computeAngles();
float fastInvSqrt(float x);

#endif /* IMU_ALGORITHMS_MAIN_H_ */

/*****************************END OF FILE*****************************/
