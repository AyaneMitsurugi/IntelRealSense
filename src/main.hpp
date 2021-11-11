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

/* AHRS Madgwick-Mahony-related parameters */
extern float are_angles_computed;

/* FUNCTIONS */
/* Return buffer of current date and time in YYYY-MM-DD-HH:mm:ss format */
const std::string currentDateTime();
