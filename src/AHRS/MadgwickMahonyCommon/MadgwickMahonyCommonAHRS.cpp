/* DEFINES */
#ifndef _MADGWICK_MAHONY_COMMON_AHRS_C_
#define _MADGWICK_MAHONY_COMMON_AHRS_C_

/* INCLUDES */
#include <math.h>
#include "MadgwickMahonyCommonAHRS.h"
#include "../Madgwick/MadgwickAHRS.h"
#include "../Mahony/MahonyAHRS.h"

/* FUNCTIONS */
void printGyroscopeMeasurements(void){
	printf("Gyroscope - measurements:\n");
	printf("gx = %f, gy = %f, gz = %f", gx, gy, gz);
}

void printfAccelerometerMeasurements(void) {
	printf("Accelerometer - measurements:\n");
	printf("ax = %f, ay = %f, az = %f", ax, ay, az);
}
void printfMagnetometerMeasurements(void) {
	printf("Magnetometer - measurements:\n");
	printf("mx = %f, my = %f, mz = %f", mx, my, mz);
}

float getRoll() {
	if (!are_angles_computed)
		computeAngles();
	return roll * 57.29578f;
}

float getPitch() {
	if (!are_angles_computed)
		computeAngles();
	return pitch * 57.29578f;
}

float getYaw() {
	if (!are_angles_computed)
		computeAngles();
	return yaw * 57.29578f + 180.0f;
}

float getRollRadians() {
	if (!are_angles_computed)
		computeAngles();
	return roll;
}

float getPitchRadians() {
	if (!are_angles_computed)
		computeAngles();
	return pitch;
}

float getYawRadians() {
	if (!are_angles_computed)
		computeAngles();
	return yaw;
}

void printfRoll(void) {
	printf("Roll: %f\n", getRoll());
}
void printfPitch(void) {
	printf("Pitch: %f\n", getPitch());
}
void printfYaw(void) {
	printf("Yaw: %f\n", getYaw());
}

void printfQuaternions(void) {
	printf("Quaternions:\n");
	printf("q0 = %f, q1 = %f, q2 = %f, q3 = %f", q0, q1, q2, q2);
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void computeAngles() {
	roll   = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	are_angles_computed = 1;
}
#endif /* _MADGWICK_MAHONY_COMMON_AHRS_C_ */

