#include <TimerOne.h>
#include <AML_PSX.h>
#include <AML_ControlMotor.h>
#include <AML_ControlPID.h>
#include <AML_IMU.h>
#include <AML_IRSensor.h>

#define dataPin 10  // Brown wire
#define cmdPin 11   // Orange wire
#define attPin 12   // Yellow wire
#define clockPin 13 // Blue wire

PSX psx;

PSX::PSXDATA PSXdata;
int PSXerror;
bool Square_Button_Pressed = false;
bool angleControlActive = false;
bool Triangle_Button_Pressed = false;
bool angle90Active = false;
bool Cirle_Button_Pressed = false;
bool angleFollowActive = false;

unsigned long lastPSXUpdateTime = 0;
unsigned long psxUpdateInterval = 50; // 50ms cho PSX
unsigned long lastIMUUpdateTime = 0;
unsigned long imuUpdateInterval = 10; // 10ms cho IMU
void updateIMU();
void AML_PSX_Setup()
{
    // Setup PSX library
    psx.setupPins(dataPin, cmdPin, attPin, clockPin, 5);
    psx.config(PSXMODE_ANALOG);
    Serial.begin(115200);
    Timer1.initialize(10000);
    Timer1.attachInterrupt(updateIMU);
}

void AML_PSX_ControlMotor()
{
    if (millis() - lastPSXUpdateTime >= psxUpdateInterval)
    {
        lastPSXUpdateTime = millis();
        PSXerror = psx.read(PSXdata);

        if (PSXerror == PSXERROR_SUCCESS)
        {
            AML_PSX_HandleControl();
        }
        else
        {
            AML_ControlMotor_PWM_Left(0, 0);
            AML_ControlMotor_PWM_Right(0, 0);
            Serial.println("No success reading data. Check connections and timing.");
        }
    }
}
void AML_PSX_HandleControl()
{

    if (PSXdata.JoyRightY < 128)
    {
        AML_ControlMotor_PWM_Left((128 - PSXdata.JoyRightY), (128 - PSXdata.JoyRightY));
        AML_ControlMotor_PWM_Right((128 - PSXdata.JoyRightY), (128 - PSXdata.JoyRightY));
    }
    else if (PSXdata.JoyRightY > 128)
    {
        AML_ControlMotor_PWM_Left(-(PSXdata.JoyRightY - 128), -(PSXdata.JoyRightY - 128));
        AML_ControlMotor_PWM_Right(-(PSXdata.JoyRightY - 128), -(PSXdata.JoyRightY - 128));
    }
    else if (PSXdata.JoyRightX > 128)
    {
        AML_ControlMotor_PWM_Left((PSXdata.JoyRightX - 128) + 20, -(PSXdata.JoyRightX - 128) - 20);
        AML_ControlMotor_PWM_Right(-(PSXdata.JoyRightX - 128) - 20, (PSXdata.JoyRightX - 128) + 20);
    }
    else if (PSXdata.JoyRightX < 128)
    {
        AML_ControlMotor_PWM_Left((PSXdata.JoyRightX - 128) - 20, abs(PSXdata.JoyRightX - 128) + 20);
        AML_ControlMotor_PWM_Right(abs(PSXdata.JoyRightX - 128) + 20, (PSXdata.JoyRightX - 128) - 20);
    }
    else
    {
        AML_ControlMotor_PWM_Left(0, 0);
        AML_ControlMotor_PWM_Right(0, 0);
    }

    Serial.print("JoyLeft: X: ");
    Serial.print(PSXdata.JoyLeftX);
    Serial.print(", Y: ");
    Serial.print(PSXdata.JoyLeftY);

    Serial.print(", JoyRight: X: ");
    Serial.print(PSXdata.JoyRightX);
    Serial.print(", Y: ");
    Serial.print(PSXdata.JoyRightY);

    Serial.print(", Buttons: ");

    if (PSXdata.buttons & PSXBTN_SQUARE)
    {
        if (!Square_Button_Pressed)
        {
            angleControlActive = !angleControlActive;
            Square_Button_Pressed = true;
            Serial.print("Toggled angle control: ");
            Serial.println(angleControlActive ? "ON" : "OFF");
        }
    }
    else
    {
        Square_Button_Pressed = false;
    }

    if (angleControlActive)
    {
        AML_ControlPID_ReturnToInitialAngle();
        Serial.print("Executing LOCK ANGLE ");
    }

    if (PSXdata.buttons & PSXBTN_CROSS)
    {
        AML_IMU_ResetAngleZ();
    }

    if (PSXdata.buttons & PSXBTN_TRIANGLE)
    {
        if (!Triangle_Button_Pressed)
        {
            angle90Active = !angle90Active;
            Triangle_Button_Pressed = true;
            Serial.print("Toggled angle control: ");
            Serial.println(angle90Active ? "ON" : "OFF");
        }
    }
    else
    {
        Triangle_Button_Pressed = false;
    }

    if (angle90Active)
    {
        AML_ControlPID_Routate90D();
        Serial.print("90 Degree ");
    }

    if (PSXdata.buttons & PSXBTN_CIRCLE)
    {
        if (!Cirle_Button_Pressed)
        {
            angleFollowActive = !angleFollowActive;
            Cirle_Button_Pressed = true;
            Serial.print("Toggled angle control: ");
            Serial.println(angleFollowActive ? "ON" : "OFF");
        }
    }
    else
    {
        Cirle_Button_Pressed = false;
    }

    if (angleFollowActive)
    {
       AML_ControlPID_Straight();
        Serial.print("follow ");
    }

}

void updateIMU()
{
    AML_IMU_SerialEvent();
}