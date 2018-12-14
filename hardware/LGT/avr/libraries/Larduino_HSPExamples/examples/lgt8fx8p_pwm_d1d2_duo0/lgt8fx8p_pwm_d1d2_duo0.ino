

uint8_t deadBand = 10;

void setup() {
  // put your setup code here, to run once:
  
  // usage: pwmMode(pin, pwm_mode, freq_mode)
  // Note 1: D1 is also used for UART/TXD, which is very important for arduino 
  //    - but never mind, just feel free to use it for pwm. 
  // Note 2: PWM of D1/D2 is controlled by timer3, which not support frequency boost
  //    - so PWM_FREQ_BOOST is not valid for pwm of d1/d2
  pwmMode(D1, PWM_MODE_DUO0, PWM_FREQ_FAST, deadBand);

  // usage: pwmResolution(pin, res_bits)
  pwmResolution(D1, 7);

  // usage: pwmWrite(pin, duty)
  // note: we set resolution to 7bit, so the duty should be inside of 0 ~ 127
  pwmWrite(D1, 128 >> 2);
  pwmWrite(D2, 128 >> 2);
}

void loop() {
  // put your main code here, to run repeatedly:
}
