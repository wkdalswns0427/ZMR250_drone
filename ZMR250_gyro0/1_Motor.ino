#include <Servo.h>

Servo Motor[4];

const uint8_t MotorPin[4] = {8, 9, 10, 11};

void InitMotor()
{
  for(uint8_t i = 0; i < 4; i++) Motor[i].attach(MotorPin[i], 1000, 2000);
}

void WriteMotor(uint8_t i, int16_t raw)
{
  Motor[i].writeMicroseconds(constrain(raw, 1000, 2000));
}

void WriteAll(int16_t raw)
{
  WriteMotor(0, raw);
  WriteMotor(1, raw);
  WriteMotor(2, raw);
  WriteMotor(3, raw);
}

/*
         FRONT
  -------------------
    (CW)M0   M3(CCW)
           X
   (CCW)M1   M2(CW)
  -------------------
*/

void QuadX(int16_t Throttle, int16_t Roll, int16_t Pitch, int16_t Yaw)
{
  WriteMotor(0, 1000 + Throttle + Roll - Pitch - Yaw);
  WriteMotor(1, 1000 + Throttle + Roll + Pitch + Yaw);
  WriteMotor(2, 1000 + Throttle - Roll + Pitch - Yaw);
  WriteMotor(3, 1000 + Throttle - Roll - Pitch + Yaw);
}
