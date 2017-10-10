
uint32_t pwmFreq = 0;

void setup() {
  // put your setup code here, to run once:
  // Note 0: pwm of D1/D2 controlled by timer3, so their settings ard shared.
  
  // usage: pwmMode(pin, pwm_mode, freq_mode)
  // Note 1: D1 is also used for UART/TXD, which is very important for arduino 
  //    - but never mind, just feel free to use it for pwm. 
  // Note 2: PWM of D1/D2 is controlled by timer3, which not support frequency boost
  //    - so PWM_FREQ_BOOST is not valid for pwm of d1/d2
  pwmMode(D1, PWM_MODE_SOLO, PWM_FREQ_FAST);

  // usage: pwmResolution(pin, res_bits)
  // set pwm resolution to 7bit
  pwmFreq = pwmResolution(D1, 7);

  // usage: pwmWrite(pin, duty)
  // note: we set resolution to 7bit, so the duty should be inside of 0 ~ 127
  pwmWrite(D1, 128 >> 2);
  pwmWrite(D2, 128 >> 4);
}

void loop() {
  // put your main code here, to run repeatedly:
}
