/* DEFINES */
#ifndef _COMMON_AHRS_C_
#define _COMMON_AHRS_C_

/* INCLUDES */
#include "CommonAHRS.hpp"

/* EXTERNAL VARIABLES */
// Common
float sample_freq     = RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME;
float inv_sample_freq = (1.0f / sample_freq); // Inverse of sample frequency [s]

float roll            = 0.0f;
float pitch           = 0.0f;
float yaw             = 0.0f;

/* AHRS Madgwick-Mahony-related parameters */
// Quaternions of sensor frame relative to auxiliary frame
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float are_angles_computed = 0;

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float fastInvSqrt(float x) {
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
    roll                = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch               = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw                 = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	are_angles_computed = 1;
}

#endif /* _COMMON_AHRS_C_ */
