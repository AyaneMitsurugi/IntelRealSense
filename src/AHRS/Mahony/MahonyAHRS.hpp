/* https://github.com/PaulStoffregen/MahonyAHRS */

/* DEFINES */
#ifndef _MAHONY_AHRS_H_
#define _MAHONY_AHRS_H_

#define TWO_KP (2.0f * 0.5f) // 2 * proportional gain (Kp)
#define TWO_KI (2.0f * 0.0f) // 2 * integral gain (Ki)

/* INCLUDES */
#include "../Common/CommonAHRS.hpp"

/* EXTERNAL VARIABLES */
extern float twoKp;                                 // 2 * proportional gain (Kp)
extern float twoKi;                                 // 2 * integral gain (Ki)
extern float integralFBx, integralFBy, integralFBz; // Integral error terms scaled by integral gain (Ki)

/* FUNCTIONS PROTOTYPES */
void MahonyPrintfIntegralErrorTerms(void);

void MahonyGyroscopeAccelerometerMagnetometer(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyGyroscopeAccelerometer(float gx, float gy, float gz, float ax, float ay, float az);

#endif /* _MAHONY_AHRS_H_ */
