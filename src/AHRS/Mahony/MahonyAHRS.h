/* https://github.com/PaulStoffregen/MahonyAHRS */

/* DEFINES */
#ifndef _MAHONY_AHRS_H_
#define _MAHONY_AHRS_H_

#define TWO_KP  (2.0f * 0.5f)  // 2 * proportional gain (Kp)
#define TWO_KI  (2.0f * 0.0f)  // 2 * integral gain (Ki)

/* INCLUDES */
#include "../../main_Madgwick_Mahony.h"

/* FUNCTIONS */
void MahonyPrintfIntegralErrorTerms(void);

void MahonyGyroscopeAccelerometerMagnetometer(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float inv_sample_freq, float are_angles_computed);
void MahonyGyroscopeAccelerometer(float gx, float gy, float gz, float ax, float ay, float az, float inv_sample_freq, float are_angles_computed);

#endif /* _MAHONY_AHRS_H_ */

