#ifndef AML_ControlPID_h
#define AML_ControlPID_h

#include <Arduino.h>

#include <AML_ControlPID.h>
#include <AML_PSX.h>
#include <AML_ControlMotor.h>
#include <AML_IMU.h>

struct AML_ControlPID_Struct
{
    double Kp;
    double Ki;
    double Kd;
    double tau;
    double sampleTime;

    double integratol;
    double prevError;
    double differentiator;
    double prevMeasurement;
    double out;

    double limMax;
    double limMin;

    double linMaxInt;
    double linMinInt;

    unsigned long lastTime;
};
extern AML_ControlPID_Struct pid;
// extern AML_ControlPID_Struct AML_PID_ReturnToInitialAngle;
// extern AML_ControlPID_Struct AML_PID_Straight;
void AML_ControlPID_Init(AML_ControlPID_Struct *pid, double kp, double ki, double kd, double tau, double sampleTime);
double AML_ControlPID_Compute(AML_ControlPID_Struct *pid, double measurement, double setpoint);
void AML_ControlPID_Routate90D();
void AML_ControlPID_ReturnToInitialAngle();
void AML_ControlPID_FollowAngle();
void AML_ControlPID_Straight();
void AML_ControlPID_Setup();
#endif