/* DEFINES */
#ifndef _COMMON_AHRS_H_
#define _COMMON_AHRS_H_

/* INCLUDE */
#include <math.h>
#include <librealsense2/rs.hpp>

/* EXTERNAL VARIABLES */
// Common
extern float sample_freq;
extern float inv_sample_freq; // Inverse of sample frequency [s]

extern float roll;
extern float pitch;
extern float yaw;

/* AHRS Madgwick-Mahony-related parameters */
extern float q0, q1, q2, q3; // Quaternions of sensor frame relative to auxiliary frame

extern float are_angles_computed;

/* FUNCTIONS PROTOTYPES */
float fastInvSqrt(float x);
void computeAngles();

#endif /* _COMMON_AHRS_H_ */
