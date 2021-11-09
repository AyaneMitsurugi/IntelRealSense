/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU_algorithms_main.c
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
#ifndef IMU_ALGORITHMS_MAIN_C_
#define IMU_ALGORITHMS_MAIN_C_

/* Includes ------------------------------------------------------------------*/
#include "IMU_algorithms_main.h"

/* Private variables ---------------------------------------------------------*/
// Gyroscope:
volatile float gx                  = 0.0f;
volatile float gy                  = 0.0f;
volatile float gz                  = 0.0f;

// Accelerometer:
volatile float ax                  = 0.0f;
volatile float ay                  = 0.0f;
volatile float az                  = 0.0f;

// Magnetometer:
volatile float mx                  = 0.0f;
volatile float my                  = 0.0f;
volatile float mz                  = 0.0f;

// Mahony & Madgwick - COMMON
volatile uint8_t areAnglesComputed = 0;
volatile float roll                = 0.0f;
volatile float pitch               = 0.0f;
volatile float yaw                 = 0.0f;

// Mahony & Madgwick - COMMON - Quaternions of sensor frame relative to auxiliary frame:
volatile float q0                  = 1.0f;
volatile float q1                  = 0.0f;
volatile float q2                  = 0.0f;
volatile float q3                  = 0.0f;

/* Private function implementation  ------------------------------------------*/
void use_specific_IMU_algorithm(uint8_t IMU_algorithm) {
	gx, gy, gz = gyroscopeMeasurements();
	ax, ay, ax = accelerometerMeasurements();
	mx, my, mz = magnetometerMeasurements();

	if(ENABLE_PRINTF == 1) {
		printGyroscopeMeasurements();
		printfAccelerometerMeasurements();
		printfMagnetometerMeasurements();
	}

	switch(IMU_algorithm) {
	case 1:  // HAL library TM_AHRSIMU
		IMUHALInit();
		HALGyroscopeAccelerometerMagnetometer();
		if(ENABLE_PRINTF == 1) {
			printf("***** HAL LIBRARY ALGORITHM *****\n");
			HALprintRoll();
			HALprintPitch();
			HALprintYaw();
		}
		break;
	case 2:  // Mahony Algorithm
		MahonyGyroscopeAccelerometerMagnetometer(gx, gy, gz, ax, ay, az, mx, my, mz);
		if(ENABLE_PRINTF == 1) {
			printf("***** MAHONY ALGORITHM *****\n");
			printfRoll();
			printPitch();
			printYaw();
			MahonyPrintfIntegralErrorTerms();
			printfQuaternions();
		}
		break;
	case 3:  // Madgwick Algorithm
		MadgwickGyroscopeAccelerometerMagnetometer(gx, gy, gz, ax, ay, az, mx, my, mz);
		if(ENABLE_PRINTF == 1) {
			printf("***** MADGWICK ALGORITHM *****\n");
			printfRoll();
			printPitch();
			printYaw();
			printfQuaternions();
		}
		break;
	default:
		printf("ERROR: Wrong IMU_algorithm value: %d", IMU_algorithm);
		break;
	}
}

float gyroscopeMeasurements() {
	//TODO: Obtain Gyroscope's measurements
	return gx, gy, gz;
}

float accelerometerMeasurements() {
	//TODO: Obtain Accelerometer's measurements
	return ax, ay, az;
}

float magnetometerMeasurements() {
	//TODO: Obtain Magnetometer's measurements
	return mx, my, mz;
}

void printGyroscopeMeasurements(void){
	printf("Gyroscope - measurements:\n");
	printf("gx = %d, gy = %d, gz = %d", gx, gy, gz);
}

void printfAccelerometerMeasurements(void) {
	printf("Accelerometer - measurements:\n");
	printf("ax = %d, ay = %d, az = %d", ax, ay, az);
}
void printfMagnetometerMeasurements(void) {
	printf("Magnetometer - measurements:\n");
	printf("mx = %d, my = %d, mz = %d", mx, my, mz);
}

float getRoll() {
	if (!areAnglesComputed)
		computeAngles();
	return roll * 57.29578f;
}

float getPitch() {
	if (!areAnglesComputed)
		computeAngles();
	return pitch * 57.29578f;
}

float getYaw() {
	if (!areAnglesComputed)
		computeAngles();
	return yaw * 57.29578f + 180.0f;
}

float getRollRadians() {
	if (!areAnglesComputed)
		computeAngles();
	return roll;
}

float getPitchRadians() {
	if (!areAnglesComputed)
		computeAngles();
	return pitch;
}

float getYawRadians() {
	if (!areAnglesComputed)
		computeAngles();
	return yaw;
}

void printfRoll(void) {
	printf("Roll: %d\n", getRoll());
}
void printfPitch(void) {
	printf("Pitch: %d\n", getPitch());
}
void printfYaw(void) {
	printf("Yaw: %d\n", getYaw());
}

#endif /* IMU_ALGORITHMS_MAIN_C_ */

/*****************************END OF FILE*****************************/
