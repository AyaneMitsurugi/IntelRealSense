/* INCLUDES */
#include "main.hpp"

/* VARIABLES */

// Output file with IMU-related data
std::ofstream outfile;

// Gyroscope - angular velocity [rad/s]
float gx = 0.0;
float gy = 0.0;
float gz = 0.0;

// Accelerometer - angular acceleration [rad/s^2]
float ax = 0.0;
float ay = 0.0;
float az = 0.0;

// Gyroscope - angular velocity for Fusion Algorithm [deg/s]
float fusion_gx = 0.0;
float fusion_gy = 0.0;
float fusion_gz = 0.0;

// Accelerometer - angular acceleration for Fusion Algorithm [g]
float fusion_ax = 0.0;
float fusion_ay = 0.0;
float fusion_az = 0.0;

// Magnetometer - not used in this project
float mx = 0.0;
float my = 0.0;
float mz = 0.0;

// Common
float precision = 6.0;

/* AHRS Fusion-related variables */
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

float roll_fus_deg  = 0.0;
float pitch_fus_deg = 0.0;
float yaw_fus_deg   = 0.0;

float roll_fus_rad  = 0.0;
float pitch_fus_rad = 0.0;
float yaw_fus_rad   = 0.0;

/* AHRS Madgwick-related variables */
float roll_mad_deg  = 0.0;
float pitch_mad_deg = 0.0;
float yaw_mad_deg   = 0.0;

float roll_mad_rad  = 0.0;
float pitch_mad_rad = 0.0;
float yaw_mad_rad   = 0.0;

/* AHRS Mahony-related variables */
float roll_mah_deg  = 0.0;
float pitch_mah_deg = 0.0;
float yaw_mah_deg   = 0.0;

float roll_mah_rad  = 0.0;
float pitch_mah_rad = 0.0;
float yaw_mah_rad   = 0.0;

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

/* Save headers in the output file */
void saveHeadersInOutputFile(void) {
    if (outfile.is_open()) {
        outfile << "Index,";
        outfile << "Translation X [m],Translation Y [m], Translation Z [m],";
        outfile << "Velocity X [m/s],Velocity Y [m/s], Velocity Z [m/s],";
        outfile << "Angular velocity X [rad/s], Angular velocity Y [rad/s], Angular velocity Z [rad/s],";                 // Gyroscope's raw data
        outfile << "Angular acceleration X [rad/s^2],Angular acceleration Y [rad/s^2],Angular acceleration Z [rad/s^2],"; // Accelerometer's raw data
        outfile << "Fusion Roll [degrees], Fusion Pitch [degrees],Fusion Yaw [degrees],";
        outfile << "Fusion Roll [rad], Fusion Pitch [rad],Fusion Yaw [rad],";
        outfile << "Madgwick Roll [degrees], Madgwick Pitch [degrees],Madgwick Yaw [degrees],";
        outfile << "Madgwick Roll [rad], Madgwick Pitch [rad],Madgwick Yaw [rad],";
        outfile << "Mahony Roll [degrees], Mahony Pitch [degrees],Mahony Yaw [degrees]";
        outfile << "Mahony Roll [rad], Mahony Pitch [rad],Mahony Yaw [rad]" << std::endl;
    }
}

/* Save sample index, translation and velocity in the output file */
void saveTransVelInOutputFile(int sample_idx, float trans_x, float trans_y, float trans_z, float vel_x, float vel_y, float vel_z) {
    if (outfile.is_open()) {
        outfile << sample_idx << ",";
        outfile << std::setprecision(precision) << std::fixed << trans_x << "," << trans_y << "," << trans_z << ",";
        outfile << std::setprecision(precision) << std::fixed << vel_x   << "," << vel_y   << "," << vel_z   << ",";
    }
}

/* Save gyroscope's and accelerometer's raw data in the output file */
void saveGyroAccInOutputFile(float gx, float gy, float gz, float ax, float ay, float az) {
    if (outfile.is_open()) {
	    outfile << std::setprecision(precision) << std::fixed << gx << "," << gy << "," << gz << ",";
	    outfile << std::setprecision(precision) << std::fixed << ax << "," << ay << "," << az << ",";
    }
}

/* Convert rad/s^2 to g */
/* https://stackoverflow.com/questions/6291931/how-to-calculate-g-force-using-x-y-z-values-from-the-accelerometer-in-android/44421684 */
void convertAccForFusion(float ax, float ay, float az) {
    const float g = 9.81;

    // Firstly, convert rad/s^2 to deg/s^2
    fusion_ax = FusionRadiansToDegrees(ax);
    fusion_ay = FusionRadiansToDegrees(ay);
    fusion_az = FusionRadiansToDegrees(az);

    // Secondly, divide by g = 9.81 to convert deg/s^2 to g
    fusion_ax /= g;
    fusion_ay /= g;
    fusion_az /= g;
}

/* Normalize Roll-Pitch-Yaw calculated by Fusion Algoritm */
void normalizeRollPitchYawFusion(float roll_rad, float pitch_rad, float yaw_rad) {
    roll_fus_rad  = fmod(roll_rad, (2*M_PI));
    pitch_fus_rad = fmod(pitch_rad, (2*M_PI));
    yaw_fus_rad   = fmod(yaw_rad, (2*M_PI));
}

/* Save Roll-Pitch-Yaw calculated by all Fusion Algorithm in the output file */
void saveRollPitchYawFusionInOutputFile (float roll_fus_deg, float pitch_fus_deg, float yaw_fus_deg, float roll_fus_rad, float pitch_fus_rad, float yaw_fus_rad) {
	if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << roll_fus_deg << "," << pitch_fus_deg << "," << yaw_fus_deg << ",";
        outfile << std::setprecision(precision) << std::fixed << roll_fus_rad << "," << pitch_fus_rad << "," << yaw_fus_rad << ",";
    }
}

/* Save Roll-Pitch-Yaw calculated by all Madgwick Algorithm in the output file */
void saveRollPitchYawMadgwickInOutputFile (float roll_mad_deg, float pitch_mad_deg, float yaw_mad_deg, float roll_mad_rad, float pitch_mad_rad, float yaw_mad_rad) {
	if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << roll_mad_deg << "," << pitch_mad_deg << "," << yaw_mad_deg << ",";
        outfile << std::setprecision(precision) << std::fixed << roll_mad_rad << "," << pitch_mad_rad << "," << yaw_mad_rad << ",";
    }
}

/* Save Roll-Pitch-Yaw calculated by Mahony Algorithm in the output file */
void saveRollPitchYawMahonyInOutputFile (float roll_mah_deg, float pitch_mah_deg, float yaw_mah_deg, float roll_mah_rad, float pitch_mah_rad, float yaw_mah_rad) {
	if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << roll_mah_deg << "," << pitch_mah_deg << "," << yaw_mah_deg << ",";
        outfile << std::setprecision(precision) << std::fixed << roll_mah_rad << "," << pitch_mah_rad << "," << yaw_mah_rad << std::endl;
    }
}

/* MAIN */
int main()
{
    /* Variables */

    // Initialize sample index
    int sample_idx = 1;

    // Create file with currentdate and time in its name
    std::string buffer = currentDateTime();
    outfile.open (buffer);

    /* TRY-CATCH */
    try {
        // Initialize pose stream
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

        saveHeadersInOutputFile();

        /* AHRS FUSION-RELATED FUNCTIONS */

        // Initialise gyroscope bias correction algorithm
        FusionBiasInitialise(&fusionBias, 0.5f, sample_freq); // stationary threshold = 0.5 degrees per second

        // Initialise AHRS algorithm
        FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5

        /* MAIN LOOP */
        while (true)
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();

            // Get a frame from the pose stream
            auto f = frames.first_or_default(RS2_STREAM_POSE);

            // Cast the frame to pose_frame and get its data
            auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

            // Get gyroscope's and accelerometer's raw data
            gx = pose_data.angular_velocity.x;     // [rad/s]
            gy = pose_data.angular_velocity.y;     // [rad/s]
            gz = pose_data.angular_velocity.z;     // [rad/s]
            ax = pose_data.angular_acceleration.x; // [rad/s^2]
            ay = pose_data.angular_acceleration.y; // [rad/s^2]
            az = pose_data.angular_acceleration.z; // [rad/s^2]

            saveTransVelInOutputFile(sample_idx, pose_data.translation.x, pose_data.translation.y , pose_data.translation.z, pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z);
            saveGyroAccInOutputFile(gx, gy, gz, ax, ay, az);

            /* AHRS FUSION-RELATED FUNCTIONS */
            fusion_gx  = FusionRadiansToDegrees(gx); // [deg/s]
            fusion_gy  = FusionRadiansToDegrees(gy); // [deg/s]
            fusion_gz  = FusionRadiansToDegrees(gz); // [deg/s]

	        // Calibrate gyroscope
            FusionVector3 uncalibratedGyroscope = {
                uncalibratedGyroscope.axis.x = fusion_gx,
                uncalibratedGyroscope.axis.y = fusion_gy,
                uncalibratedGyroscope.axis.z = fusion_gz,
            };
            FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

            // Accelometer - convert rad/s^2 to deg/s^2 and then to g
            convertAccForFusion(ax, ay, az); // [g]

	        // Calibrate accelerometer
            FusionVector3 uncalibratedAccelerometer = {
                uncalibratedAccelerometer.axis.x = fusion_ax,
                uncalibratedAccelerometer.axis.y = fusion_ay,
                uncalibratedAccelerometer.axis.z = fusion_az,
            };
            FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	        // Update gyroscope bias correction algorithm
            calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

            // Update AHRS algorithm
            FusionAhrsUpdateWithoutMagnetometer(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, sample_freq);

            FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));

            roll_fus_rad  = eulerAngles.angle.roll;  // [rad]
            pitch_fus_rad = eulerAngles.angle.pitch; // [rad]
            yaw_fus_rad   = eulerAngles.angle.yaw;   // [rad]

            normalizeRollPitchYawFusion(roll_fus_rad, pitch_fus_rad, yaw_fus_rad);

            roll_fus_deg  = FusionRadiansToDegrees(roll_fus_rad);  // [degrees]
            pitch_fus_deg = FusionRadiansToDegrees(pitch_fus_rad); // [degrees]
            yaw_fus_deg   = FusionRadiansToDegrees(yaw_fus_rad);   // [degrees]

            saveRollPitchYawFusionInOutputFile (roll_fus_deg, pitch_fus_deg, yaw_fus_deg, roll_fus_rad, pitch_fus_rad,  yaw_fus_rad);

            /* AHRS MADGWICK-RELATED FUNCTIONS */
            MadgwickGyroscopeAccelerometer(gx, gy, gz, ax, ay, az);
            computeAngles();

            roll_mad_rad  = roll;  // [rad]
	        pitch_mad_rad = pitch; // [rad]
	        yaw_mad_rad   = yaw;   // [rad]

            roll_mad_deg  = FusionRadiansToDegrees(roll_mad_rad);  // [degrees]
            pitch_mad_deg = FusionRadiansToDegrees(pitch_mad_rad); // [degrees]
            yaw_mad_deg   = FusionRadiansToDegrees(yaw_mad_rad);   // [degrees]

            saveRollPitchYawMadgwickInOutputFile (roll_mad_deg, pitch_mad_deg, yaw_mad_deg, roll_mad_rad, pitch_mad_rad, yaw_mad_rad);

            /* AHRS MAHONY-RELATED FUNCTIONS */
            MahonyGyroscopeAccelerometer(gx, gy, gz, ax, ay, az);
            computeAngles();

            roll_mah_rad  = roll;  // [rad]
	        pitch_mah_rad = pitch; // [rad]
	        yaw_mah_rad   = yaw;   // [rad]

            roll_mah_deg  = FusionRadiansToDegrees(roll_mah_rad);  // [degrees]
            pitch_mah_deg = FusionRadiansToDegrees(pitch_mah_rad); // [degrees]
            yaw_mah_deg   = FusionRadiansToDegrees(yaw_mah_rad);   // [degrees]

            saveRollPitchYawMahonyInOutputFile(roll_mah_deg, pitch_mah_deg, yaw_mah_deg, roll_mah_rad, pitch_mah_rad, yaw_mah_rad);

            std::cout << "Saving " << sample_idx << "th sample in the " << buffer << " file \r";

            // Increase sample index
            sample_idx++;
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
