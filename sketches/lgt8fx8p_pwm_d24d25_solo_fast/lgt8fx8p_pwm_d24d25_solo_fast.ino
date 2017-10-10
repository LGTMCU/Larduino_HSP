
uint16_t dutyMax = 0;

void setup() {
  // put your setup code here, to run once:
  
  // usage: pwmMode(pin, pwm_mode, freq_mode)
  // Note 1: PWM of D24/D25 is back-up of D9/D10, 
  //      - so in case of D9/D10 must used for other purpose(e,g, SPI), we can switch their pwm function
  //      - to D24/D25 (E4/E5) which is not exist in stardard arduino board.
  pwmMode(D25, PWM_MODE_SOLO, PWM_FREQ_FAST|PWM_FREQ_BOOST);

  // usage: pwmFrequency(pin, freq_in_hz)
  // set to 2MHz PWM output
  dutyMax = pwmFrequency(D25, 2000000);

  // usage: pwmWrite(pin, duty)
  // note: we set resolution to 7bit, so the duty should be inside of 0 ~ 1024
  pwmWrite(D25, dutyMax >> 1);
  pwmWrite(D24, dutyMax >> 2);
}

void loop() {
  // put your main code here, to run repeatedly:
}
