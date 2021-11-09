/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU_Mahony.c
  * @brief          : Implementation of Mahony Algorithm.
  ******************************************************************************
  * @description
  *
  * Link: https://github.com/PaulStoffregen/MahonyAHRS
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_MAHONY_C_
#define IMU_MAHONY_C_

/* Includes ------------------------------------------------------------------*/
#include "IMU_Mahony.h"
#include "IMU_algorithms_main.h"

/* Private variables ---------------------------------------------------------*/
volatile float twoKp       = TWO_KP;  // 2 * proportional gain (Kp)
volatile float twoKi       = TWO_KI;  // 2 * integral gain (Ki)

// Integral error terms scaled by integral gain (Ki):
volatile float integralFBx = 0.0f;
volatile float integralFBy = 0.0f;
volatile float integralFBz = 0.0f;

/* Private function implementation  ------------------------------------------*/


void MahonyPrintfIntegralErrorTerms(void) {
	printf("Integral error terms scaled by integral gain (Ki):\n");
	printf("FBx = %d, FBy = %d, FBz = %d", integralFBx, integralFBy, integralFBz);
}

void MahonyGyroscopeAccelerometerMagnetometer(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float normalization;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use algorithm without magnetometer when its measurement are invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyGyroscopeAccelerometer(gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer's measurement are valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer's measurement
		normalization = fastInvSqrt(ax*ax + ay*ay + az*az);
		ax           *= normalization;
		ay           *= normalization;
		az           *= normalization;

		// Normalise magnetometer's measurement
		normalization = fastInvSqrt(mx*mx + my*my + mz*mz);
		mx           *= normalization;
		my           *= normalization;
		mz           *= normalization;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx*hx + hy*hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// If integral feedback is enabled
		if(twoKi > 0.0f) {
			// Integral error terms scaled by integral gain (Ki):
			integralFBx += twoKi * halfex * INV_SAMPLE_FREQ;
			integralFBy += twoKi * halfey * INV_SAMPLE_FREQ;
			integralFBz += twoKi * halfez * INV_SAMPLE_FREQ;

			// Apply integral feedback:
			gx          += integralFBx;
			gy          += integralFBy;
			gz          += integralFBz;
		}
		// If integral feedback is disabled prevent integral windup
		else {
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}  // END OF: if(twoKi > 0.0f)

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}  // END OF: if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))

	// Integrate rate of change of quaternions
	// Pre-multiply common factors:
	gx *= (0.5f * INV_SAMPLE_FREQ);
	gy *= (0.5f * INV_SAMPLE_FREQ);
	gz *= (0.5f * INV_SAMPLE_FREQ);

	qa  = q0;
	qb  = q1;
	qc  = q2;

	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternions
	normalization = fastInvSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0           *= normalization;
	q1           *= normalization;
	q2           *= normalization;
	q3           *= normalization;

	areAnglesComputed = 0;
}

void MahonyGyroscopeAccelerometer(float gx, float gy, float gz, float ax, float ay, float az) {
	float normalization;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer's measurement are valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer's measurement
		normalization = fastInvSqrt(ax*ax + ay*ay + az*az);
		ax           *= normalization;
		ay           *= normalization;
		az           *= normalization;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// If integral feedback is enabled
		if(twoKi > 0.0f) {
			// Integral error terms scaled by integral gain (Ki):
			integralFBx += twoKi * halfex * INV_SAMPLE_FREQ;
			integralFBy += twoKi * halfey * INV_SAMPLE_FREQ;
			integralFBz += twoKi * halfez * INV_SAMPLE_FREQ;

			// Apply integral feedback:
			gx          += integralFBx;
			gy          += integralFBy;
			gz          += integralFBz;
		}
		// If integral feedback is disabled prevent integral windup
		else {
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}  // END OF: if(twoKi > 0.0f)

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}  // END OF: if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))

	// Integrate rate of change of quaternions
	// Pre-multiply common factors:
	gx *= (0.5f * INV_SAMPLE_FREQ);
	gy *= (0.5f * INV_SAMPLE_FREQ);
	gz *= (0.5f * INV_SAMPLE_FREQ);

	qa  = q0;
	qb  = q1;
	qc  = q2;

	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternions
	normalization = fastInvSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0           *= normalization;
	q1           *= normalization;
	q2           *= normalization;
	q3           *= normalization;

	areAnglesComputed = 0;
}

#endif /* IMU_MAHONY_C_ */

/*****************************END OF FILE*****************************/
