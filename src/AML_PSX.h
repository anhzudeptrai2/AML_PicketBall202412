#ifndef AML_PSX_h
#define AML_PSX_h

#include <Arduino.h>
#include <PSX.h>
#include <AML_ControlPID.h>
#include <AML_IMU.h>
#include <AML_ControlMotor.h>

void AML_PSX_Setup();
void AML_PSX_ControlMotor();
void AML_PSX_HandleControl();

#endif