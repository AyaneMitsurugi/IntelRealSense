// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include "IntelRealSenseDemo/example-utils.hpp"
#include "AHRS/Fusion.h"
#include "AHRS/FusionAhrs.h"
#include "AHRS/FusionBias.h"
#include "AHRS/FusionCalibration.h"
#include "AHRS/FusionTypes.h"

FusionBias fusionBias;
FusionAhrs fusionAhrs;

float samplePeriod = RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME;

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

int main()
{
    try {
        std::string serial;
        if (!device_with_streams({ RS2_STREAM_POSE}, serial))
            return EXIT_SUCCESS;

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;

        // Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;
        if (!serial.empty())
            cfg.enable_device(serial);

        // Add pose stream
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

        // Start pipeline with chosen configuration
        pipe.start(cfg);

        // Initialise gyroscope bias correction algorithm
        FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod); // stationary threshold = 0.5 degrees per second

        // Initialise AHRS algorithm
        FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5

    /***** MAIN LOOP *****/
        while (true)
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();

            // Get a frame from the pose stream
            auto f = frames.first_or_default(RS2_STREAM_POSE);

            // Cast the frame to pose_frame and get its data
            auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

            // Print the x, y, z values of the translation, relative to initial position
            std::cout << "Device Position X " << std::setprecision(3) << std::fixed << pose_data.translation.x << " Y " << pose_data.translation.y << " Z " << pose_data.translation.z << " [m]\n";

	    // Calibrate gyroscope
            FusionVector3 uncalibratedGyroscope = {
                uncalibratedGyroscope.axis.x = pose_data.translation.x,
                uncalibratedGyroscope.axis.y = pose_data.translation.y,
                uncalibratedGyroscope.axis.z = pose_data.translation.z,
            };
            FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

	    // Calibrate accelerometer
            FusionVector3 uncalibratedAccelerometer = {
                uncalibratedAccelerometer.axis.x = pose_data.angular_acceleration.x,
                uncalibratedAccelerometer.axis.y = pose_data.angular_acceleration.y,
                uncalibratedAccelerometer.axis.z = pose_data.angular_acceleration.z,
            };
            FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	    // Update gyroscope bias correction algorithm
            calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

            // Update AHRS algorithm
            FusionAhrsUpdateWithoutMagnetometer(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, samplePeriod);

	    // Print Euler angles
            FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
	    std::cout << "Roll " << std::setprecision(3) << std::fixed << eulerAngles.angle.roll << " Pitch " << eulerAngles.angle.pitch << " Yaw " << eulerAngles.angle.yaw << " [degrees]\n";
        }
        return EXIT_SUCCESS;
    }
    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

