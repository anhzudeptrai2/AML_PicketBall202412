#include <Arduino.h>

uint8_t Re_buf[33], position = 0;
uint32_t sign = 0;
float g[3] = {0}, Current_AngleZ = 0;
float Save_AngleZ = 0, Prev_AngleZ = 0;

void AML_IMU_Read();

void AML_IMU_ResetAngleZ()
{
    Serial1.write(0xFF);
    Serial1.write(0xAA);
    Serial1.write(0x52);
    Save_AngleZ = 0;
    Current_AngleZ = 0;
    Prev_AngleZ = 0;
    delay(5);
}

void AML_IMU_Setup()
{
    Serial1.begin(115200);
    Serial.begin(115200);
    AML_IMU_ResetAngleZ();
}
void AML_IMU_SerialEvent()
{
    while (Serial1.available())
    {
        uint8_t byte_in = Serial1.read();
        if (position == 0 && byte_in != 0x55)
        {
            position = 0; //
            continue;
        }
        Re_buf[position] = byte_in;
        position++;
        if (position == 33)
        {
            position = 0;
            sign = 1;
            AML_IMU_Read();
        }
    }
}

void AML_IMU_Read()
{
    if (sign)
    {
        // Serial.println(Re_buf[23]);
        sign = 0;
        Prev_AngleZ = Current_AngleZ;
        if (Re_buf[22] == 0x55 && Re_buf[23] == 0x53)
        {
            g[2] = (((Re_buf[29] << 8 | Re_buf[28])) / 32768.0) * 180;
            Current_AngleZ = g[2];
        }

        // if (Current_AngleZ - Prev_AngleZ > 250.0f)
        // {
        //     Save_AngleZ += 360.0f;
        // }
        // else if (Current_AngleZ - Prev_AngleZ < -250.0f)
        // {
        //     Save_AngleZ -= 360.0f;
        // }
    }
    Serial.print("Angle: ");
    Serial.println(g[2]);
}
double AML_IMU_GetAngle(void)
{
    return Current_AngleZ - Save_AngleZ;
}
