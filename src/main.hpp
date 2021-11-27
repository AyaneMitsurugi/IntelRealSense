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
extern float gx_rad;
extern float gy_rad;
extern float gz_rad;

// Accelerometer
extern float ax_rad;
extern float ay_rad;
extern float az_rad;

// Gyroscope - angular velocity for Fusion Algorithm [deg/s]
extern float fusion_gx_deg;
extern float fusion_gy_deg;
extern float fusion_gz_deg;

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
void saveGyroAccInOutputFile(float gx_rad, float gy_rad, float gz_rad, float ax_rad, float ay_rad, float az_rad);

/* Convert rad/s^2 to g*/
/* https://stackoverflow.com/questions/6291931/how-to-calculate-g-force-using-x-y-z-values-from-the-accelerometer-in-android/44421684 */
void convertAccForFusion(float ax_rad, float ay_rad, float az_rad);

/* Normalize Roll-Pitch-Yaw calculated by Fusion Algoritm */
void normalizeRollPitchYawFusion(float roll_fus_rad, float pitch_fus_rad, float yaw_fus_rad);

/* Save Roll-Pitch-Yaw calculated by Fusion Algorithm in the output file */
void saveRollPitchYawFusionInOutputFile (float roll_fus_deg, float pitch_fus_deg, float yaw_fus_deg, float roll_fus_rad, float pitch_fus_rad, float yaw_fus_rad);

/* Save Roll-Pitch-Yaw calculated by Madgwick Algorithm in the output file */
void saveRollPitchYawMadgwickInOutputFile (float roll_mad_deg, float pitch_mad_deg, float yaw_mad_deg, float roll_mad_rad, float pitch_mad_rad, float yaw_mad_rad);

/* Save Roll-Pitch-Yaw calculated by Mahony Algorithm in the output file */
void saveRollPitchYawMahonyInOutputFile (float roll_mah_deg, float pitch_mah_deg, float yaw_mah_deg, float roll_mah_rad, float pitch_mah_rad, float yaw_mah_rad);

/* Save quaternions for all Algorithms in the output file */
void saveQuaternionsInOutputFile(float fus_qx, float fus_qy, float fus_qz, float fus_qw, float mad_qx, float mad_qy, float mad_qz, float mad_qw, float mah_qx, float mah_qy, float mah_qz, float mah_qw);
