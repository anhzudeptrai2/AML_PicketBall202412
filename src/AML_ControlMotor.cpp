#include <AML_ControlMotor.h>

#define RFPWM1 2 // Dong co phai truoc
#define LFPWM1 3 // Dong co phai truoc
#define RBPWM1 8 // Dong co phai sau
#define LBPWM1 9 // Dong co phai sau
#define RFPWM2 4 // Dong co trai truoc
#define LFPWM2 5 // Dong co trai truoc
#define RBPWM2 6 // Dong co trai sau
#define LBPWM2 7 // Dong co trai sau


void AML_ControlMotor_PWM_Right(int pwmL, int pwmR)
{
    if (pwmL > 0) // DC truoc tien
    {
        analogWrite(LFPWM1, pwmL);
        analogWrite(RFPWM1, 0);
    }
    else if (pwmL < 0) // DC L lui
    {
        analogWrite(LFPWM1, 0);
        analogWrite(RFPWM1, abs(pwmL));
    }
    else if (pwmL == 0) // DC Dung yen
    {
        analogWrite(LFPWM1, 0);
        analogWrite(RFPWM1, 0);
    }

    if (pwmR > 0) // Sau tiến
    {
        analogWrite(LBPWM1, pwmR);
        analogWrite(RBPWM1, 0);
    }
    else if (pwmR < 0) // R lùi
    {
        analogWrite(LBPWM1, 0);
        analogWrite(RBPWM1, abs(pwmR));
    }
    else if (pwmR == 0) // R đứng yên
    {
        analogWrite(LBPWM1, 0);
        analogWrite(RBPWM1, 0);
    }
}

void AML_ControlMotor_PWM_Left(int pwmL, int pwmR)
{
    if (pwmL > 0) // DC truoc tien
    {
        analogWrite(LFPWM2, pwmL);
        analogWrite(RFPWM2, 0);
    }
    else if (pwmL < 0) // DC L lui
    {
        analogWrite(LFPWM2, 0);
        analogWrite(RFPWM2, abs(pwmL));
    }
    else if (pwmL == 0) // DC Dung yen
    {
        analogWrite(LFPWM2, 0);
        analogWrite(RFPWM2, 0);
    }

    if (pwmR > 0) // Sau tiến
    {
        analogWrite(LBPWM2, pwmR);
        analogWrite(RBPWM2, 0);
    }
    else if (pwmR < 0) // R lùi
    {
        analogWrite(LBPWM2, 0);
        analogWrite(RBPWM2, abs(pwmR));
    }
    else if (pwmR == 0) // R đứng yên
    {
        analogWrite(LBPWM2, 0);
        analogWrite(RBPWM2, 0);
    }
}

void AML_ControlMotor_BTSsetup()
{
    pinMode(LFPWM1, OUTPUT);
    pinMode(RFPWM1, OUTPUT);
    pinMode(LBPWM1, OUTPUT);
    pinMode(RBPWM1, OUTPUT);
    pinMode(LFPWM2, OUTPUT);
    pinMode(RFPWM2, OUTPUT);
    pinMode(LBPWM2, OUTPUT);
    pinMode(RBPWM2, OUTPUT);
}
