
uint8_t deadBand = 8;

void setup() {
  // put your setup code here, to run once:
  
  // usage: pwmMode(pin, pwm_mode, freq_mode, dead_band)
  pwmMode(D9, PWM_MODE_DUO1, PWM_FREQ_FAST|PWM_FREQ_BOOST, deadBand);

  // usage: pwmResolution(pin, res_bits)
  pwmResolution(D9, 7);

  // usage:  pwmWrite(pin,  duty_ration)
  // note: for our 7bit pwm resolution, duty should be inside of (1 << 7) = 128
  pwmWrite(D9, 100);
  pwmWrite(D10, 100);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
