#ifndef AML_IMU_h

#include <Arduino.h>

void AML_IMU_Setup();
void AML_IMU_SerialEvent();
void AML_IMU_Read();
void AML_IMU_ResetAngleZ();
float AML_IMU_GetAngle();

#endif

