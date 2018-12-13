
uint16_t dutyMax = 0xff;

void setup() {
  // put your setup code here, to run once:
  
  // usage: pwmMode(pin, pwm_mode, freq_mode)
  // Note: PWM of D3/D11 is controlled by timer2, which is used for system tick (delay)
  //      - so it's not allowed to change its frequency, we can only get its default settings.
  //      - pwmMode() settings will do nothing for D3/D11's frequency,
  //      - the more, timer2 do not support complementary mode, so only PWM_MODE_SOLO is supported.
  //        PWM_MODE_SOLO: d3/d11 working in independed mode 
  // In one word:
  //    pwm of D3/D11 frequency is fixed to 970KHz (as stardard arduino)
  //    only duty ratio can be modified, between 0 ~ 255
  pwmMode(D3, PWM_MODE_DUO1);

  // usage: pwmWrite(pin, duty)
  pwmWrite(D3, dutyMax >> 2);
  pwmWrite(D11, dutyMax >> 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}
