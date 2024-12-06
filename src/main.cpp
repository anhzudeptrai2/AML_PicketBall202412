#include <Arduino.h>
#include <AML_ControlMotor.h>
#include <AML_PSX.h>
#include <AML_IMU.h>
#include <AML_ControlPID.h>
#include <AML_IRSensor.h>
#include <PSX.h>

void setup()
{
  Serial1.begin(115200);
  Serial.begin(115200);
  AML_PSX_Setup();
  AML_ControlMotor_BTSsetup();
  AML_ControlMotor_PWM_Left(0, 0);
  AML_ControlMotor_PWM_Right(0, 0);
  AML_IMU_Setup();
  AML_ControlPID_Setup();

  // Cấu hình Timer1
  cli();                               // Tắt ngắt toàn cục để cấu hình timer
  TCCR1A = 0;                          // Chế độ Normal
  TCCR1B = (1 << WGM12);               // Chế độ CTC (Clear Timer on Compare Match)
  TIMSK1 = (1 << OCIE1A);              // Cho phép ngắt khi Timer1 tràn
  OCR1A = 2499;                        // Đặt giá trị so sánh để tạo ngắt mỗi 10ms (16MHz / 64 / 2500)
  TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler = 64

  sei(); // Bật ngắt toàn cục
}

void loop()
{

  AML_PSX_ControlMotor();
  AML_IMU_SerialEvent();
 
  // AML_ControlPID_Routate90D();
}
