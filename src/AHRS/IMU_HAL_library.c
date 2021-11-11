/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU_HAL_library.c
  * @brief          : Usage of HAL library TM_AHRSIMU methods
  ******************************************************************************
  * @description
  *
  * Usage of HAL library TM_AHRSIMU methods, which calculates Roll, Pitch and
  * Yaw angles by using AHRS and IMU algorithms.
  * Link: https://stm32f4-discovery.net/hal_api/group___t_m___a_h_r_s_i_m_u.html
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_HAL_LIBRARY_C_
#define IMU_HAL_LIBRARY_C_

/* Includes ------------------------------------------------------------------*/
#include "IMU_HAL_library.h"

/* Private variables ---------------------------------------------------------*/
TM_AHRSIMU_t AHRSIMU;

/* Private function implementation  ------------------------------------------*/
void IMUHALInit(void) {
	TM_AHRSIMU_Init(&AHRSIMU, SAMPLE_FREQ, BETA, CRACOV_INCLINATION);
}

void HALGyroscopeAccelerometerMagnetometer(void) {
	float gx, gy, gz;  // Gyroscope
	float ax, ay, az;  // Accelerometer
	float mx, my, mz;  // Magnetometer

	/* Gyroscope with +-2000 degrees/s on 16-bits
	    -2000 = -32768 (16-bits)
	     2000 =  32767 (16-bits)
	   Accelerometer with +-16 G on 16 bits
	   -16 = -32768 (16-bits)
	    16 =  32767 (16-bits) */

	/* Convert data to gees, deg/sec and microTesla respectively */
	gx = (gx * GYRO_MAX) / BITS_MAX;
	gy = (gy * GYRO_MAX) / BITS_MAX;
	gz = (gz * GYRO_MAX) / BITS_MAX;

	ax = (ax * ACC_MAX) / BITS_MAX;
	ay = (ay * ACC_MAX) / BITS_MAX;
	az = (az * ACC_MAX) / BITS_MAX;

	/* Call update function */
	TM_AHRSIMU_UpdateAHRS(&AHRSIMU, AHRSIMU_DEG2RAD(gx), AHRSIMU_DEG2RAD(gy), AHRSIMU_DEG2RAD(gz), ax, ay, az, mx, my, mz);
}

void HALprintRoll(void) {
	printf("Roll: %d\n", AHRSIMU.Roll);
}

void HALprintPitch(void) {
	printf("Pitch: %d\n", AHRSIMU.Pitch);
}

void HALprintYaw(void) {
	printf("Yaw: %d\n", AHRSIMU.Yaw);
}

#endif /* IMU_HAL_LIBRARY_C_ */

/*****************************END OF FILE*****************************/
