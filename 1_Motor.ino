#include <Servo.h>

Servo Motor[4];
double MotorSpeed[4] = {0};

const uint8_t MotorPin[4] = {8, 9, 10, 11};

void InitMotor()
{
  for(uint8_t i = 0; i < 4; i++) Motor[i].attach(MotorPin[i], 500, 2000);
}

void WriteMotor(uint8_t i, int16_t raw)
{
  Motor[i].writeMicroseconds(constrain(raw, 500, 2000));
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
  uint8_t j;
  MotorSpeed[0] = 720 + Throttle - Roll - Pitch - Yaw;
  MotorSpeed[1] = 1020 + Throttle - Roll + Pitch + Yaw; // pitch issues
  MotorSpeed[2] = 920 + Throttle + Roll + Pitch - Yaw;
  MotorSpeed[3] = 970 + Throttle + Roll - Pitch + Yaw;

  WriteMotor(0, MotorSpeed[0]);
  WriteMotor(1, MotorSpeed[1]);
  WriteMotor(2, MotorSpeed[2]);
  WriteMotor(3, MotorSpeed[3]);
  
  for(j=0;j<4;j++){
    Serial.print(MotorSpeed[j]);
    Serial.print("\t");
  }
  Serial.println("\n");
}
