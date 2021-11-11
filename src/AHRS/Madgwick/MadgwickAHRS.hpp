/* https://github.com/PaulStoffregen/MadgwickAHRS */

/* DEFINES */
#ifndef _MADGWICK_AHRS_H_
#define _MADGWICK_AHRS_H_

#define BETA 0.1f // 2 * proportional gain (Kp)

/* INCLUDES */
#include "../Common/CommonAHRS.hpp"

/* EXTERNL VARIABLES */
extern float beta;           // 2 * proportional gain (Kp)

/* FUNCTIONS PROTOTYPES */
void MadgwickGyroscopeAccelerometerMagnetometer(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickGyroscopeAccelerometer(float gx, float gy, float gz, float ax, float ay, float az);

#endif /* _MADGWICK_AHRS_H_ */
