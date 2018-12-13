
uint16_t freq = 0;
uint8_t pwm_flag = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(D2, INPUT_PULLUP);

  // set default status after PWM turn off
  digitalWrite(D5, LOW);
  digitalWrite(D6, HIGH);
  
  // usage: pwmMode(pin, pwm_mode, freq_mode)
  pwmMode(D5, PWM_MODE_SOLO, PWM_FREQ_FAST);

  // usage: pwmResolution(pin, res_bits)
  freq = pwmResolution(D5, 7);

  // usage: pwmWrite(pin, duty)
  pwmWrite(D5, 128 >> 1);
  pwmWrite(D6, 128 >> 2);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(D2) == LOW && pwm_flag == 0) {
    pwm_flag = 1;
    pwmTurnOff(D5);
    pwmTurnOff(D6);
  } else if(digitalRead(D2) == HIGH && pwm_flag == 1) {
    pwm_flag = 0;
    pwmWrite(D5, 128 >> 1);
    pwmWrite(D6, 128 >> 2);
  }
}
