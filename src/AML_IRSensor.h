#ifndef AML_IRSensor_h
#define AML_IRSensor_h

#include <Arduino.h>

void AML_IRSensor_Setup();
int32_t AML_IRSensor_Read();
void AML_IRSensor_Control(int motor, int speed);


#endif
