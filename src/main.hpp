/* INCLUDES */
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <librealsense2/rs.hpp>
#include "IntelRealSenseDemo/example-utils.hpp"
#include "AHRS/Fusion/Fusion.h"
#include "AHRS/Fusion/FusionAhrs.h"
#include "AHRS/Fusion/FusionBias.h"
#include "AHRS/Fusion/FusionCalibration.h"
#include "AHRS/Fusion/FusionTypes.h"
#include "AHRS/Madgwick/MadgwickAHRS.hpp"
#include "AHRS/Mahony/MahonyAHRS.hpp"
#include "AHRS/Common/CommonAHRS.hpp"

/* EXTERNAL VARIABLES */
// Gyroscope
extern float gx;
extern float gy;
extern float gz;

// Accelerometer
extern float ax;
extern float ay;
extern float az;

// Gyroscope - angular velocity for Fusion Algorithm [deg/s]
extern float fusion_gx;
extern float fusion_gy;
extern float fusion_gz;

// Accelerometer - angular acceleration for Fusion Algorithm [g]
extern float fusion_ax;
extern float fusion_ay;
extern float fusion_az;

// Magnetometer - not used in this project
extern float mx;
extern float my;
extern float mz;

// Common
extern float precision;

extern float roll;
extern float pitch;
extern float yaw;

/* AHRS Fusion-related parameters */
extern FusionBias fusionBias;
extern FusionAhrs fusionAhrs;

extern FusionVector3 gyroscopeSensitivity;
extern FusionVector3 accelerometerSensitivity;

/* AHRS Fusion-related parameters */
extern float roll_fus_deg;
extern float pitch_fus_deg;
extern float yaw_fus_deg;

extern float roll_fus_rad;
extern float pitch_fus_rad;
extern float yaw_fus_rad;

/* AHRS Madgwick-related parameters */
extern float roll_mad_deg;
extern float pitch_mad_deg;
extern float yaw_mad_deg;

extern float roll_mad_rad;
extern float pitch_mad_rad;
extern float yaw_mad_rad;

/* AHRS Mahony-related functions */
extern float roll_mah_deg;
extern float pitch_mah_deg;
extern float yaw_mah_deg;

extern float roll_mah_rad;
extern float pitch_mah_rad;
extern float yaw_mah_rad;

/* FUNCTIONS */
/* Return buffer of current date and time in YYYY-MM-DD-HH:mm:ss format */
const std::string currentDateTime();

/* Save headers in the output file */
void saveHeadersInOutputFile(void);

/* Save sample index, translation and velocity in the output file */
void saveTransVelInOutputFile(int sample_idx, float trans_x, float trans_y, float trans_z, float vel_x, float vel_y, float vel_z);

/* Save gyroscope's and accelerometer's raw data in the output file */
void saveGyroAccInOutputFile(float gx, float gy, float gz, float ax, float ay, float az);

/* Convert rad/s^2 to g*/
/* https://stackoverflow.com/questions/6291931/how-to-calculate-g-force-using-x-y-z-values-from-the-accelerometer-in-android/44421684 */
void convertAccForFusion(float ax, float ay, float az);

/* Normalize Roll-Pitch-Yaw calculated by Fusion Algoritm */
void normalizeRollPitchYawFusion(float roll_fus_rad, float pitch_fus_rad, float yaw_fus_rad);

/* Save Roll-Pitch-Yaw calculated by Fusion Algorithm in the output file */
void saveRollPitchYawFusionInOutputFile (float roll_fus_deg, float pitch_fus_deg, float yaw_fus_deg, float roll_fus_rad, float pitch_fus_rad, float yaw_fus_rad);

/* Save Roll-Pitch-Yaw calculated by Madgwick Algorithm in the output file */
void saveRollPitchYawMadgwickInOutputFile (float roll_mad_deg, float pitch_mad_deg, float yaw_mad_deg, float roll_mad_rad, float pitch_mad_rad, float yaw_mad_rad);

/* Save Roll-Pitch-Yaw calculated by Mahony Algorithm in the output file */
void saveRollPitchYawMahonyInOutputFile (float roll_mah_deg, float pitch_mah_deg, float yaw_mah_deg, float roll_mah_rad, float pitch_mah_rad, float yaw_mah_rad);
