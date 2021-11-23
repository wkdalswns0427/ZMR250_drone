volatile bool chLast[6] = {0};
volatile int16_t chData[6] = {0};
volatile uint32_t chTime = 0;

void setup() {
  Serial.begin(115200);
  InitIMU();
  InitMotor();
  PCICR |= (1 << PCIE2); // PCICR (pin change interrupt control register)
  PCMSK2 |= (1 << PCINT2); // Mask for bit0  2
  PCMSK2 |= (1 << PCINT3); // Mask for bit1  3
  PCMSK2 |= (1 << PCINT4); // Mask for bit2  4
  PCMSK2 |= (1 << PCINT5); // Mask for bit3  5
  PCMSK2 |= (1 << PCINT6); // Mask for bit3  6
  PCMSK2 |= (1 << PCINT7); // Mask for bit3  7
}

void loop() {
  UpdateIMU();
  UpdatePID();
}

ISR(PCINT2_vect) {
  uint32_t nowTime = micros();
  /* Channel */
  for(uint8_t i = 0; i < 6; i++)
  {
    bool Channel = digitalRead(2+i);
    if(chLast[i] != Channel)
    {
      if(Channel) chTime = nowTime; // Rising
      else chData[i] = constrain(nowTime - chTime, 1000, 2000) - 1000; // Falling
      chLast[i] = Channel;
    }
  }
}
