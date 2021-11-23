#define ROLL_STABLE_KP 4
#define ROLL_STABLE_KI 0.01
#define ROLL_RATE_KP   100
#define ROLL_RATE_KI   0
#define ROLL_RATE_KD   9

#define PITCH_STABLE_KP ROLL_STABLE_KP
#define PITCH_STABLE_KI ROLL_STABLE_KI
#define PITCH_RATE_KP   ROLL_RATE_KP
#define PITCH_RATE_KI   ROLL_RATE_KI
#define PITCH_RATE_KD   ROLL_RATE_KD

#define YAW_STABLE_KP 3
#define YAW_STABLE_KI 0
#define YAW_RATE_KP 4
#define YAW_RATE_KI 0
#define YAW_RATE_KD 3

double ITerm[3][2] = {0,}, lastRate[3] = {0,};
double ANGLE[3] = {0,}, RATE[3] = {0,};
double rcData[6] = {0,};
bool Drive = false, Debug = false;

void UpdateData()
{
  // 0 ~ 1000 -> -10 ~ 10
  if(Drive)
  {
    rcData[0] = (chData[1] - 500) / 50.0; // Roll
    rcData[1] = (chData[2] - 500) / 50.0; // Pitch
    double Temp = (chData[0] - 500) / 1000.0;
    if(fabs(Temp) < 0.2) Temp = 0;
    rcData[2] += Temp; // Yaw
    rcData[3] = chData[2]; // Throttle
  }
  rcData[4] = chData[4] > 500; // SW A
  rcData[5] = chData[5] > 500; // SW D
  // Angle&Gyro
  ANGLE[0] = -Angle[0];
  ANGLE[1] = -Angle[1];
  ANGLE[2] = -Angle[2];
  RATE[0] = -Rate[0];
  RATE[1] = -Rate[1];
  RATE[2] = -Rate[2];
  // Switch
  Debug = rcData[5];
  if(rcData[3] < 50 || Debug) Drive = rcData[4];
  // Debug
  /*
  if(Debug)
  {
    Serial.print(ANGLE[0]);
    Serial.print("\t");
    Serial.print(ANGLE[1]);
    Serial.print("\t");
    Serial.print(ANGLE[2]);
    Serial.print("\t");
    Serial.print(rcData[2]);
    Serial.print("\t");
    Serial.println(DT, 4);
  }
  */
}

void UpdatePID()
{
  UpdateData();
  // PID Tunes
  double PTerm, DTerm, errRate, errAngle, Control[3];
  if(Drive)
  {
    /* ROLL */
    errAngle = rcData[0] - ANGLE[0];
    errRate = (errAngle * ROLL_STABLE_KP) - (RATE[0] * ROLL_RATE_KP);
    PTerm = errRate;
    ITerm[0][0] += errAngle * ROLL_STABLE_KI;
    ITerm[0][1] += errRate * ROLL_RATE_KI;
    DTerm = (errRate - lastRate[0]) * ROLL_RATE_KD;
    lastRate[0] = errRate;
    Control[0] = PTerm + ITerm[0][0] + ITerm[0][1] + DTerm;
    /* PITCH */
    errAngle = rcData[1] - ANGLE[1];
    errRate = (errAngle * PITCH_STABLE_KP) - (RATE[1] * PITCH_RATE_KP);
    PTerm = errRate;
    ITerm[1][0] += errAngle * PITCH_STABLE_KI;
    ITerm[1][1] += errRate * PITCH_RATE_KI;
    DTerm = (errRate - lastRate[1]) * PITCH_RATE_KD;
    lastRate[1] = errRate;
    Control[1] = PTerm + ITerm[1][0] + ITerm[1][1] + DTerm;
    /* YAW */
    errAngle = rcData[2] - ANGLE[2]; // 목표값- ANGLE[2];
    errRate = (errAngle * YAW_STABLE_KP) - (RATE[2] * YAW_RATE_KP);
    PTerm = errRate;
    // ITerm[2][0] += errAngle * YAW_STABLE_KI;
    ITerm[2][1] += errRate * YAW_RATE_KI;
    DTerm = (errRate - lastRate[2]) * YAW_RATE_KD;
    lastRate[2] = errRate;
    Control[2] = PTerm + ITerm[2][0] + ITerm[2][1] + DTerm;
    /* Min&Max Cut */
    ITerm[0][0] = constrain(ITerm[0][0], -10, 10);
    ITerm[0][1] = constrain(ITerm[0][1], -10, 10);
    ITerm[1][0] = constrain(ITerm[1][0], -10, 10);
    ITerm[1][1] = constrain(ITerm[1][1], -10, 10);
    ITerm[2][0] = constrain(ITerm[2][0], -10, 10);
    ITerm[2][1] = constrain(ITerm[2][1], -10, 10);
    Control[0] = constrain(Control[0], -200, 200);
    Control[1] = constrain(Control[1], -200, 200);
    Control[2] = constrain(Control[2], -100, 100);
    /* Motor Write */
    QuadX(50 + rcData[3], Control[0], Control[1], Control[2]);
  }
  else WriteAll(1000);
}
