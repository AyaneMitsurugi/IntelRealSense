/* DEFINES */
#ifndef _MADGWICK_MAHONY_COMMON_AHRS_H_
#define _MADGWICK_MAHONY_COMMON_AHRS_H_

/* INCLUDES */
#include <stdio.h>
#include "../../main_Madgwick_Mahony.h"

/* Functions prototypes ------------------------------------------------------*/
float gyroscopeMeasurements();
float accelerometerMeasurements();
float magnetometerMeasurements();

void printGyroscopeMeasurements(void);
void printfAccelerometerMeasurements(void);
void printfMagnetometerMeasurements(void);

float gtRoll();
float getPitch();
float getYaw();

float getRollRadians();
float getPitchRadians();
float getYawRadians();

void printfRoll(void);
void printfPitch(void);
void printfYaw(void);

void printfQuaternions(void);
void computeAngles();
float fastInvSqrt(float x);

#endif /* _MADGWICK_MAHONY_COMMON_AHRS_H_ */

