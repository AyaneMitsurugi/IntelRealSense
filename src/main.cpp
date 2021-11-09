/* INCLUDES */
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <time.h>
#include <librealsense2/rs.hpp>
#include "IntelRealSenseDemo/example-utils.hpp"
#include "AHRS/Fusion/Fusion.h"
#include "AHRS/Fusion/FusionAhrs.h"
#include "AHRS/Fusion/FusionBias.h"
#include "AHRS/Fusion/FusionCalibration.h"
#include "AHRS/Fusion/FusionTypes.h"
#include "AHRS/Madgwick/MadgwickAHRS.h"
#include "AHRS/Mahony/MahonyAHRS.h"

float precision    = 6.0;
float gx = 0.0;
float gy = 0.0;
float gz = 0.0;
float ax = 0.0;
float ay = 0.0;
float az = 0.0;

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

/* AHRS Madgwick-related parameters */

/* AHRS Mahony-related parameters */

/* FUNCTIONS */
/* Return buffer of current date and time in YYYY-MM-DD-HH:mm:ss format */
const std::string currentDateTime() {
    time_t    now     = time(0);
    struct tm tstruct;
    char      buffer [80];
    tstruct           = *localtime(&now);

    strftime(buffer, sizeof(buffer), "../output_data/%Y-%m-%d-%X", &tstruct);

    return buffer;
};

/* MAIN */
int main()
{
    /* Variables */
    int index = 1;

    std::string buffer = currentDateTime();
    std::ofstream outfile;
    outfile.open (buffer);

    try {
        // TODO
        std::string serial;
        if (!device_with_streams({RS2_STREAM_POSE}, serial))
            return EXIT_SUCCESS;

        // Create RealSense pipeline (encapsulating the actual device and sensors)
        rs2::pipeline pipe;

        // Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;
        if (!serial.empty())
            cfg.enable_device(serial);

        // Add pose stream
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

        // Start pipeline with chosen configuration
        pipe.start(cfg);

        // Output file - save headers
        if (outfile.is_open()) {
            outfile << "Index,";
            outfile << "Translation X [m],Translation Y [m], Translation Z [m],";
            outfile << "Velocity X [m/s], Velocity Y [m/s], Velocity Z [m/s],";
            outfile << "Angular acceleration X [rad/s^2],Angular acceleration Y [rad/s^2],Angular acceleration Z [rad/s^2],";
            outfile << "Roll [degrees],Pitch[degrees],Yaw[degrees]" << std::endl;
        }

        /* AHRS Fusion-related functions */
        // Initialise gyroscope bias correction algorithm
        FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod); // stationary threshold = 0.5 degrees per second
        // Initialise AHRS algorithm
        FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5

        /* AHRS Madgwick-related functions */

        /* AHRS Mahony-related functions */
        
        /***** MAIN LOOP *****/
        while (true)
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();

            // Get a frame from the pose stream
            auto f = frames.first_or_default(RS2_STREAM_POSE);

            // Cast the frame to pose_frame and get its data
            auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

            // Get raw sensors' data
            gx = pose_data.translation.x;
            gy = pose_data.translation.y;
            gz = pose_data.translation.z;
            ax = pose_data.angular_acceleration.x;
            ay = pose_data.angular_acceleration.y;
            az = pose_data.angular_acceleration.z;

            // Output file - save raw sensors' data
            if (outfile.is_open()) {
                outfile << index << ",";
                outfile << std::setprecision(precision) << std::fixed << gx << "," << gy << "," << gz << ",";
	        outfile << std::setprecision(precision) << std::fixed << pose_data.velocity.x << "," << pose_data.velocity.y << "," << pose_data.velocity.z << ",";
	        outfile << std::setprecision(precision) << std::fixed << ax << "," << ay << "," << az << ",";
            }

            /* AHRS Fusion-related functions */
	    // Calibrate gyroscope
            FusionVector3 uncalibratedGyroscope = {
                uncalibratedGyroscope.axis.x = gx,
                uncalibratedGyroscope.axis.y = gy,
                uncalibratedGyroscope.axis.z = gz,
            };
            FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

	    // Calibrate accelerometer
            FusionVector3 uncalibratedAccelerometer = {
                uncalibratedAccelerometer.axis.x = ax,
                uncalibratedAccelerometer.axis.y = ay,
                uncalibratedAccelerometer.axis.z = az,
            };
            FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	    // Update gyroscope bias correction algorithm
            calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

            // Update AHRS algorithm
            FusionAhrsUpdateWithoutMagnetometer(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, samplePeriod);

            FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));

            /* AHRS Madgwick-related functions */
            Madgwick();
            MadgwickUpdateIMU(gx, gy, gz, ax, ay, az);
            MadgwickComputeAngles();

            /* AHRS Mahony-related functions */

	    // Output file - save roll-pitch-yaw calculated by all three algorithms
	    if (outfile.is_open()) {
                outfile << std::setprecision(precision) << std::fixed << eulerAngles.angle.roll << "," << eulerAngles.angle.pitch << "," << eulerAngles.angle.yaw << std::endl;
            }
            index++;
        } // END OF: while (true)
        outfile.close();
        return EXIT_SUCCESS;
    } // END OF: try
    catch (const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    } // END OF: try-catch
} // END OF: main()
