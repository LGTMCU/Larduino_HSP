
uint16_t dutyMax = 0xff;
uint8_t deadBand = 0x8;

void setup() {
  // put your setup code here, to run once:
  
  // usage: pwmMode(pin, pwm_mode, freq_mode)
  // Note: D5/D6 do not support modify pwm frequency directly,
  //      - We can adjust its frequency range to one of SLOW/NORMAL and FAST range
  //      - PWM_FREQ_FAST mode = 62.5KHz, can be boost up to 250KHz (boost x4)
  //      - PWM_FREQ_NORMAL mode = 976Hz, can be boost up to 3.9KHz (boost x4)
  //      - PWM_FREQ_SLOW mode = 61Hz, can be boost up to 244Hz (boost x4)
  // demo will set pwm to FAST mode with boost, so get 250KHz output
  pwmMode(D5, PWM_MODE_DUO1, PWM_FREQ_FAST|PWM_FREQ_BOOST, deadBand);

  // usage: pwmWrite(pin, duty)
  pwmWrite(D5, dutyMax >> 2);
  pwmWrite(D6, dutyMax >> 2);
}

void loop() {
  // put your main code here, to run repeatedly:
}
