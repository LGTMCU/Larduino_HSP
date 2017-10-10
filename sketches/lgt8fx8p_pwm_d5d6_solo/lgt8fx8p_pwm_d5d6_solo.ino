
uint16_t dutyMax = 0xff;
uint8_t deadBand = 0x8;

void setup() {
  // put your setup code here, to run once:

 // usage: pwmMode(pin, pwm_mode, freq_mode)
  pwmMode(D5, PWM_MODE_SOLO, PWM_FREQ_FAST|PWM_FREQ_BOOST);

  // usage: pwmFrequency(pin, freq_in_hz)
  // 250KHz PWM frequency
  dutyMax = pwmFrequency(D5, 250000);

  // usage: pwmWrite(pin, duty)
  pwmWrite(D5, dutyMax >> 2);
  pwmWrite(D6, dutyMax >> 2);
}

void loop() {
  // put your main code here, to run repeatedly:
}
