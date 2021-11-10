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
#include "AHRS/Madgwick/MadgwickAHRS.h"
#include "AHRS/Mahony/MahonyAHRS.h"

/* PRIVATE VARIABLES */
// Gyroscope
float gx = 0.0;
float gy = 0.0;
float gz = 0.0;

// Accelerometer
float ax = 0.0;
float ay = 0.0;
float az = 0.0;

// Magnetometer - not used in this project
float mx = 0.0;
float my = 0.0;
float mz = 0.0;

float precision = 6.0;

/* AHRS Fusion-related parameters */
float samplePeriod = RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME;
FusionBias fusionBias;
FusionAhrs fusionAhrs;

FusionVector3 gyroscopeSensitivity = {
    gyroscopeSensitivity.axis.x = 2000.0f,
    gyroscopeSensitivity.axis.y = 2000.0f,
    gyroscopeSensitivity.axis.z = 2000.0f,
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

FusionVector3 accelerometerSensitivity = {
    accelerometerSensitivity.axis.x = 16.0f,
    accelerometerSensitivity.axis.y = 16.0f,
    accelerometerSensitivity.axis.z = 16.0f,
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

/* AHRS Madgwick-Mahony-related parameters */
extern float inv_sample_freq     = (1.0f / samplePeriod);  // Inverse of sample frequency [s]
extern float are_angles_computed = 0;
extern float roll                = 0.0f;
extern float pitch               = 0.0f;
extern float yaw                 = 0.0f;

/* FUNCTIONS */
/* Return buffer of current date and time in YYYY-MM-DD-HH:mm:ss format */
const std::string currentDateTime();
