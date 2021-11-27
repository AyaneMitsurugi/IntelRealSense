/* INCLUDES */
#include "main.hpp"

/* VARIABLES */

// Output file with IMU-related data
std::ofstream outfile;

// Gyroscope - angular velocity [rad/s]
float gx_rad = 0.0;
float gy_rad = 0.0;
float gz_rad = 0.0;

// Gyroscope - angular velocity for Fusion Algorithm [deg/s]
float fusion_gx_deg = 0.0;
float fusion_gy_deg = 0.0;
float fusion_gz_deg = 0.0;

// Accelerometer - angular acceleration [rad/s^2]
float ax_rad = 0.0;
float ay_rad = 0.0;
float az_rad = 0.0;

// Accelerometer - angular acceleration for Fusion Algorithm [g-force]
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

float fus_qx = 0.0;
float fus_qy = 0.0;
float fus_qz = 0.0;
float fus_qw = 0.0;

/* AHRS Madgwick-related variables */
float roll_mad_deg  = 0.0;
float pitch_mad_deg = 0.0;
float yaw_mad_deg   = 0.0;

float roll_mad_rad  = 0.0;
float pitch_mad_rad = 0.0;
float yaw_mad_rad   = 0.0;

float mad_qx = 0.0;
float mad_qy = 0.0;
float mad_qz = 0.0;
float mad_qw = 0.0;

/* AHRS Mahony-related variables */
float roll_mah_deg  = 0.0;
float pitch_mah_deg = 0.0;
float yaw_mah_deg   = 0.0;

float roll_mah_rad  = 0.0;
float pitch_mah_rad = 0.0;
float yaw_mah_rad   = 0.0;

float mah_qx = 0.0;
float mah_qy = 0.0;
float mah_qz = 0.0;
float mah_qw = 0.0;

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
        outfile << "Translation X [m],Translation Y [m],Translation Z [m],";
        outfile << "Velocity X [m/s],Velocity Y [m/s],Velocity Z [m/s],";
        outfile << "Angular velocity X [rad/s],Angular velocity Y [rad/s],Angular velocity Z [rad/s],";                 // Gyroscope's raw data
        outfile << "Angular acceleration X [rad/s^2],Angular acceleration Y [rad/s^2],Angular acceleration Z [rad/s^2],"; // Accelerometer's raw data
        outfile << "Fusion Roll [degrees],Fusion Pitch [degrees],Fusion Yaw [degrees],";
        outfile << "Fusion Roll [rad],Fusion Pitch [rad],Fusion Yaw [rad],";
        outfile << "Madgwick Roll [degrees],Madgwick Pitch [degrees],Madgwick Yaw [degrees],";
        outfile << "Madgwick Roll [rad],Madgwick Pitch [rad],Madgwick Yaw [rad],";
        outfile << "Mahony Roll [degrees],Mahony Pitch [degrees],Mahony Yaw [degrees]";
        outfile << "Mahony Roll [rad],Mahony Pitch [rad],Mahony Yaw [rad]";
        outfile << "Fusion qx,Fusion qy,Fusion qz,Fusion qw";
        outfile << "Madgwick qx,Madgwick qy,Madgwick qz,Madgwick qw";
        outfile << "Mahony qx,Mahony qy,Mahony qz,Mahony qw" << std::endl;
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
void saveGyroAccInOutputFile(float gx_rad, float gy_rad, float gz_rad, float ax_rad, float ay_rad, float az_rad) {
    if (outfile.is_open()) {
	    outfile << std::setprecision(precision) << std::fixed << gx_rad << "," << gy_rad << "," << gz_rad << ",";
	    outfile << std::setprecision(precision) << std::fixed << ax_rad << "," << ay_rad << "," << az_rad << ",";
    }
}

/* Convert rad/s^2 to g */
/* https://stackoverflow.com/questions/6291931/how-to-calculate-g-force-using-x-y-z-values-from-the-accelerometer-in-android/44421684 */
void convertAccForFusion(float ax_rad, float ay_rad, float az_rad) {
    const float g = 9.81;

    // Firstly, convert rad/s^2 to deg/s^2
    fusion_ax = FusionRadiansToDegrees(ax_rad);
    fusion_ay = FusionRadiansToDegrees(ay_rad);
    fusion_az = FusionRadiansToDegrees(az_rad);

    // Secondly, divide by g = 9.81 to convert deg/s^2 to g
    fusion_ax /= g;
    fusion_ay /= g;
    fusion_az /= g;
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
        outfile << std::setprecision(precision) << std::fixed << roll_mah_rad << "," << pitch_mah_rad << "," << yaw_mah_rad << ",";
    }
}

/* Save quaternions for all Algorithms in the output file */
void saveQuaternionsInOutputFile(float fus_qx, float fus_qy, float fus_qz, float fus_qw, float mad_qx, float mad_qy, float mad_qz, float mad_qw, float mah_qx, float mah_qy, float mah_qz, float mah_qw) {
	if (outfile.is_open()) {
        outfile << std::setprecision(precision) << std::fixed << fus_qx << "," << fus_qy << "," << fus_qz << "," << fus_qw << ",";
        outfile << std::setprecision(precision) << std::fixed << mad_qx << "," << mad_qy << "," << mad_qz << "," << mad_qw << ",";
        outfile << std::setprecision(precision) << std::fixed << mah_qx << "," << mah_qy << "," << mah_qz << "," << mah_qw << std::endl;
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
            gx_rad = pose_data.angular_velocity.x;
            gy_rad = pose_data.angular_velocity.y;
            gz_rad = pose_data.angular_velocity.z;
            ax_rad = pose_data.angular_acceleration.x;
            ay_rad = pose_data.angular_acceleration.y;
            az_rad = pose_data.angular_acceleration.z;

            saveTransVelInOutputFile(sample_idx, pose_data.translation.x, pose_data.translation.y , pose_data.translation.z, pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z);
            saveGyroAccInOutputFile(gx_rad, gy_rad, gz_rad, ax_rad, ay_rad, az_rad);

            /* AHRS FUSION-RELATED FUNCTIONS */

            // Convert [rad] -> [degrees]
            fusion_gx_deg  = FusionRadiansToDegrees(gx_rad);
            fusion_gy_deg  = FusionRadiansToDegrees(gy_rad);
            fusion_gz_deg  = FusionRadiansToDegrees(gz_rad);

	        // Calibrate gyroscope
            FusionVector3 uncalibratedGyroscope = {
                uncalibratedGyroscope.axis.x = fusion_gx_deg,
                uncalibratedGyroscope.axis.y = fusion_gy_deg,
                uncalibratedGyroscope.axis.z = fusion_gz_deg,
            };
            FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

            // Accelometer - convert rad/s^2 to deg/s^2 and then to g
            convertAccForFusion(ax_rad, ay_rad, az_rad);

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

            // Quaternions
            auto FusionQuaternion = FusionAhrsGetQuaternion(&fusionAhrs);
            fus_qx = fusionAhrs.quaternion.element.x;
            fus_qy = fusionAhrs.quaternion.element.y;
            fus_qz = fusionAhrs.quaternion.element.z;
            fus_qw = fusionAhrs.quaternion.element.w;

            FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionQuaternion);

            roll_fus_rad  = eulerAngles.angle.roll;
            pitch_fus_rad = eulerAngles.angle.pitch;
            yaw_fus_rad   = eulerAngles.angle.yaw;

            // Convert [rad] -> [degrees]
            roll_fus_deg  = FusionRadiansToDegrees(roll_fus_rad);
            pitch_fus_deg = FusionRadiansToDegrees(pitch_fus_rad);
            yaw_fus_deg   = FusionRadiansToDegrees(yaw_fus_rad);

            saveRollPitchYawFusionInOutputFile (roll_fus_deg, pitch_fus_deg, yaw_fus_deg, roll_fus_rad, pitch_fus_rad,  yaw_fus_rad);

            /* AHRS MADGWICK-RELATED FUNCTIONS */
            MadgwickGyroscopeAccelerometer(gx_rad, gy_rad, gz_rad, ax_rad, ay_rad, az_rad);
            computeAngles();

            // Quaternions
            mad_qx = q1;
            mad_qy = q2;
            mad_qz = q3;
            mad_qw = q0;

            roll_mad_rad  = roll;
	        pitch_mad_rad = pitch;
	        yaw_mad_rad   = yaw;

            // Convert [rad] -> [degrees]
            roll_mad_deg  = FusionRadiansToDegrees(roll_mad_rad);
            pitch_mad_deg = FusionRadiansToDegrees(pitch_mad_rad);
            yaw_mad_deg   = FusionRadiansToDegrees(yaw_mad_rad);

            saveRollPitchYawMadgwickInOutputFile (roll_mad_deg, pitch_mad_deg, yaw_mad_deg, roll_mad_rad, pitch_mad_rad, yaw_mad_rad);

            /* AHRS MAHONY-RELATED FUNCTIONS */
            MahonyGyroscopeAccelerometer(gx_rad, gy_rad, gz_rad, ax_rad, ay_rad, az_rad);
            computeAngles();

            // Quaternions
            mah_qx = q1;
            mah_qy = q2;
            mah_qz = q3;
            mah_qw = q0;

            roll_mah_rad  = roll;
	        pitch_mah_rad = pitch;
	        yaw_mah_rad   = yaw;

            // Convert [rad] -> [degrees]
            roll_mah_deg  = FusionRadiansToDegrees(roll_mah_rad);
            pitch_mah_deg = FusionRadiansToDegrees(pitch_mah_rad);
            yaw_mah_deg   = FusionRadiansToDegrees(yaw_mah_rad);

            saveRollPitchYawMahonyInOutputFile(roll_mah_deg, pitch_mah_deg, yaw_mah_deg, roll_mah_rad, pitch_mah_rad, yaw_mah_rad);
            saveQuaternionsInOutputFile(fus_qx, fus_qy, fus_qz, fus_qw, mad_qx, mad_qy, mad_qz, mad_qw, mah_qx, mah_qy, mah_qz, mah_qw);

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
