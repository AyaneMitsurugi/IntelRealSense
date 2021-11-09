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
#include "MadgwickMahonyCommon.h"

/* Private function implementation  ------------------------------------------*/
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
	if (!are_anglesComputed)
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
