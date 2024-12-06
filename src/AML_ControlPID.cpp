
#include <AML_IMU.h>
#include <AML_ControlMotor.h>
#include <avr/io.h>
#include <avr/interrupt.h>

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

volatile double Current_Angle = 0;
volatile bool Update_PID = 0;

AML_ControlPID_Struct pid;

AML_ControlPID_Struct AML_PID_ReturnToInitialAngle;
double AML_PID_ReturnToInitialAngle_Kp = 0.25;
double AML_PID_ReturnToInitialAngle_Ki = 0.2;
double AML_PID_ReturnToInitialAngle_Kd = 0.5;
double AML_PID_ReturnToInitialAngle_tau = 1;

AML_ControlPID_Struct AML_PID_Straight;
double AML_PID_Straight_Kp = 1;
double AML_PID_Straight_Ki = 0.2;
double AML_PID_Straight_Kd = 0.5;
double AML_PID_Straight_tau = 1;

double Scale_PWM(double output, double error)
{
  if (abs(error) > 10.0f)
  {
    return output;
  }
  if (abs(error) > 5.0f)
  {
    return output * 0.2f;
  }
  return 0.5f;
}
double Process_Angle(double Angle)
{
  while (Angle > 180)
  {
    Angle -= 360.0f;
  }
  while (Angle < -180)
  {
    Angle += 360.0f;
  }
  return Angle;
}

double Dynamic_Error(double error)
{
  if (abs(error) > 10.0f)
  {
    return 5.0f;
  }
  if (abs(error) > 5.0f)
  {
    return 2.0f;
  }
  return 0.5f;
}
void AML_ControlPID_Init(AML_ControlPID_Struct *pid, double kp, double ki, double kd, double tau, double sampleTime)
{
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->tau = tau;
  pid->sampleTime = sampleTime;

  pid->integratol = 0;
  pid->prevError = 0;

  pid->differentiator = 0;
  pid->prevMeasurement = 0;

  pid->out = 0;

  pid->limMax = 200;    // PWM max value
  pid->limMin = -200;   // PWM min value
  pid->linMaxInt = 255; // Limit integrator
  pid->linMinInt = -255;
  pid->lastTime = millis();
}

double AML_ControlPID_Compute(AML_ControlPID_Struct *pid, double measurement, double setpoint)
{
  unsigned long now = millis();
  unsigned long timeChange = now - pid->lastTime;

  if (timeChange >= pid->sampleTime)
  {
    double error = setpoint - measurement;
    double pTerm = pid->Kp * error;

    pid->integratol += error * pid->sampleTime;
    pid->integratol += 0.5 * pid->Ki * pid->sampleTime * (error + pid->prevError);

    if (pid->integratol > pid->linMaxInt)
      pid->integratol = pid->linMaxInt;
    else if (pid->integratol < pid->linMinInt)
      pid->integratol = pid->linMinInt;

    double iTerm = pid->Ki * pid->integratol;

    pid->differentiator = -(2.0 * pid->Kd * (measurement - pid->prevMeasurement) +
                            (2.0 * pid->tau - pid->sampleTime) * pid->differentiator) /
                          (2.0 * pid->tau + pid->sampleTime);

    double dTerm = pid->Kd * pid->differentiator;

    pid->out = pTerm + iTerm + dTerm;

    if (pid->out > pid->limMax)
      pid->out = pid->limMax;
    else if (pid->out < pid->limMin)
      pid->out = pid->limMin;

    pid->prevMeasurement = measurement;
    pid->prevError = error;
    pid->lastTime = now;
  }
  return pid->out;
}

void AML_ControlPID_Setup()
{
  AML_ControlPID_Init(&pid, 2, 0.25, 6.5, 0.02, 100);
  AML_ControlPID_Init(&AML_PID_ReturnToInitialAngle, AML_PID_ReturnToInitialAngle_Kp, AML_PID_ReturnToInitialAngle_Ki, AML_PID_ReturnToInitialAngle_Kd, AML_PID_ReturnToInitialAngle_tau, 100);
  AML_ControlPID_Init(&AML_PID_Straight, AML_PID_Straight_Kp, AML_PID_Straight_Ki, AML_PID_Straight_Kd, AML_PID_Straight_tau, 100);
}

ISR(TIMER1_COMPA_vect)
{
  Current_Angle = AML_IMU_GetAngle();
  Update_PID = true;
}
void AML_ControlPID_Routate90D()
{

  static double Target_Angle = Current_Angle + 90.0f;
  // static unsigned long Start_Time = millis();

  if (Update_PID)
  {
    double output = AML_ControlPID_Compute(&pid, Current_Angle, Target_Angle);
    Serial.print(output);
    AML_ControlMotor_PWM_Left(-output, -output);
    AML_ControlMotor_PWM_Right(output, output);

    if (abs(Current_Angle - Target_Angle) < 4.0f)
    {
      AML_ControlMotor_PWM_Left(0, 0);
      AML_ControlMotor_PWM_Right(0, 0);
      return;
    }

    Update_PID = false;
  }
}
void AML_ControlPID_Straight()
{
  static double Target_Angle = 0.0;
  static double Base_PWM = 155;
  if (Update_PID)
  {

    double error = Process_Angle(Target_Angle - Current_Angle);

    double output = AML_ControlPID_Compute(&pid, Current_Angle, Target_Angle);

    if (abs(error) > 1.0f)
    {

      if (error > 0.0f) // Left
      {
        Serial.print(output);
        AML_ControlMotor_PWM_Left(output, output);
        AML_ControlMotor_PWM_Right(-output, -output);
      }
      else
      {
        Serial.print(output);
        AML_ControlMotor_PWM_Left(+output, +output);
        AML_ControlMotor_PWM_Right(-output, -output);
      }
    }
    else
    {

      AML_ControlMotor_PWM_Left(Base_PWM, Base_PWM);
      AML_ControlMotor_PWM_Right(Base_PWM, Base_PWM);
    }

    Update_PID = false;
  }
}

void AML_ControlPID_ReturnToInitialAngle()
{
  static double Initial_Angle = 0;
  // static unsigned long Start_Time = millis();
  static double Last_Error = 0;
  static unsigned long Stable_Time = millis();

  if (Update_PID)
  {
    double error = Process_Angle(Initial_Angle - Current_Angle);
    double output = AML_ControlPID_Compute(&AML_PID_ReturnToInitialAngle, Current_Angle, Initial_Angle);
    output = Scale_PWM(output, error);

    if (abs(error) > Dynamic_Error(error))
    {
      if (error < 0.00f)
      {
        AML_ControlMotor_PWM_Left(-output, -output);
        AML_ControlMotor_PWM_Right(output, output);
      }
      else
      {
        AML_ControlMotor_PWM_Left(output, output);
        AML_ControlMotor_PWM_Right(-output, -output);
      }
    }
    else
    {
      AML_ControlMotor_PWM_Left(0, 0);
      AML_ControlMotor_PWM_Right(0, 0);
      Update_PID = false;
      return;
    }

    if (abs(error - Last_Error) < 1) // Tăng ngưỡng sai số ổn định
    {
      if (millis() - Stable_Time > 2000) // Giảm thời gian ổn định
      {
        AML_ControlMotor_PWM_Left(0, 0);
        AML_ControlMotor_PWM_Right(0, 0);
      }
    }
    else
    {
      Stable_Time = millis();
    }

    Last_Error = error;

    // if (millis() - Start_Time > 7000) // Tăng thời gian cho phép
    // {
    //   AML_ControlMotor_PWM_Left(0, 0);
    //   AML_ControlMotor_PWM_Right(0, 0);
    // }
  }
}
