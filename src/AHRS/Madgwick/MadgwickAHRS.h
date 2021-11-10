/* Link: https://github.com/PaulStoffregen/MadgwickAHRS */

/* DEFINES */
#ifndef _MADGWICK_AHRS_H_
#define _MADGWICK_AHRS_H_

#define BETA          0.1f             // 2 * proportional gain (Kp)

/* INCLUDES */


/* VARIABLES */
extern volatile float beta;            // 2 * proportional gain (Kp)
extern volatile float q0, q1, q2, q3;  // Quaternions of sensor frame relative to auxiliary frame

/* FUNCTIONS */
void MadgwickGyroscopeAccelerometerMagnetometer(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float inv_sample_freq, float are_angles_computed);
void MadgwickGyroscopeAccelerometer(float gx, float gy, float gz, float ax, float ay, float az, float inv_sample_freq, float are_angles_computed);
#endif /* _MADGWICK_AHRS_H_ */

