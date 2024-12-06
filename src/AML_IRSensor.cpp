#include <Arduino.h>

const int lineSensorPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};
int sensorValues[8];

int R_PWM_1 = 2, L_PWM_1 = 3;
int R_PWM_2 = 4, L_PWM_2 = 5;

int R_PWM_3 = 6, L_PWM_3 = 7;
int R_PWM_4 = 8, L_PWM_4 = 9;

int error = 0, previousError = 0;
float KP = 10, KI = 0, KD = 0;

int PIDvalue = 0;
int baseSpeed = 25;
int maxSpeed = 255;

void AML_IRSensor_Setup()
{
    Serial.begin(9600);

    for (int i = 0; i < 8; i++)
    {
        pinMode(lineSensorPins[i], INPUT);
    }

    pinMode(R_PWM_1, OUTPUT);
    pinMode(L_PWM_1, OUTPUT);

    pinMode(R_PWM_2, OUTPUT);
    pinMode(L_PWM_2, OUTPUT);

    pinMode(R_PWM_3, OUTPUT);
    pinMode(L_PWM_3, OUTPUT);

    pinMode(R_PWM_4, OUTPUT);
    pinMode(L_PWM_4, OUTPUT);
}

int32_t AML_IRSensor_Read()
{
    int sumPositions = 0;
    int activeSensors = 0;

    for (int i = 0; i < 8; i++)
    {
        int value = digitalRead(lineSensorPins[i]);
        if (value == HIGH)
        {
            sumPositions += i;
            activeSensors++;
        }
    }

    if (activeSensors > 0)
    {
        int averagePosition = sumPositions / activeSensors;
        return averagePosition - 3.5;
    }
    else
    {
        return previousError;
    }
}

void AML_IRSensor_Control(int motor, int speed)
{
    int R_PWM, L_PWM;

    if (motor == 1)
    {
        R_PWM = R_PWM_1;
        L_PWM = L_PWM_1;
    }
    else if (motor == 2)
    {
        R_PWM = R_PWM_2;
        L_PWM = L_PWM_2;
    }
    else if (motor == 3)
    {
        R_PWM = R_PWM_3;
        L_PWM = L_PWM_3;
    }
    else if (motor == 4)
    {
        R_PWM = R_PWM_4;
        L_PWM = L_PWM_4;
    }
    else
        return;

    speed = constrain(speed, -255, 255);
    if (speed > 0)
    {
        analogWrite(R_PWM, speed);
        analogWrite(L_PWM, 0);
    }
    else if (speed < 0)
    {
        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, -speed);
    }
    else
    {
        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 0);
    }
}
void AML_IRSensor_PIDline(int pTerm, int iTerm, int dTerm)
{
    error = AML_IRSensor_Read();
    pTerm = error;
    iTerm = iTerm + error;
    dTerm = error - previousError;

    PIDvalue = (KP * pTerm) + (KI * iTerm) + (KD * dTerm);
    previousError = error;

    int leftSpeed = baseSpeed - PIDvalue;
    int rightSpeed = baseSpeed + PIDvalue;

    leftSpeed = constrain(leftSpeed, 0, maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, maxSpeed);
    Serial.print("righSpeed: ");
    Serial.print(rightSpeed);
    Serial.print(", ");
    Serial.print("leftSpeed: ");
    Serial.println(leftSpeed);
    AML_IRSensor_Control(1, rightSpeed);
    AML_IRSensor_Control(2, leftSpeed);

    AML_IRSensor_Control(3, leftSpeed);
    AML_IRSensor_Control(4, rightSpeed);
}
