#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define FWD LOW
#define BWD HIGH
#define tonePin 6

void setupMotor()
{
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
}

void car_move(float power_in)
{
  if (power_in >= 0)
  {
    digitalWrite(L_DIR_PIN, FWD);
    digitalWrite(R_DIR_PIN, FWD);

    analogWrite(L_PWM_PIN, power_in);
    analogWrite(R_PWM_PIN, power_in);

  }
  else
  {
    digitalWrite(L_DIR_PIN, BWD);
    digitalWrite(R_DIR_PIN, BWD);

    power_in = -power_in;
    analogWrite(L_PWM_PIN, power_in);
    analogWrite(R_PWM_PIN, power_in);

  }
}

void car_wheel(float power_in)
{
  if (power_in >= 0)
  {
    //turn left once power >= 0
    digitalWrite(L_DIR_PIN, BWD);
    digitalWrite(R_DIR_PIN, FWD);

    analogWrite(L_PWM_PIN, power_in);
    analogWrite(R_PWM_PIN, power_in);
  }
  else
  {
    digitalWrite(L_DIR_PIN, FWD);
    digitalWrite(R_DIR_PIN, BWD);

    power_in = -power_in;
    analogWrite(L_PWM_PIN, power_in);
    analogWrite(R_PWM_PIN, power_in);
  }
}


void car_motor(float left_power, float right_power)
{
  if (left_power >= 0)
  {
    digitalWrite(L_DIR_PIN, FWD);
    analogWrite(L_PWM_PIN, left_power);
  }
  else
  {
    left_power = -left_power;
    digitalWrite(L_DIR_PIN, BWD);
    analogWrite(L_PWM_PIN, left_power);
  }

  if (right_power >= 0)
  {
    digitalWrite(R_DIR_PIN, FWD);
    analogWrite(R_PWM_PIN, right_power);
  }
  else
  {
    right_power = -right_power;
    digitalWrite(R_DIR_PIN, BWD);
    analogWrite(R_PWM_PIN, right_power);
  }

}

void beep(int beep_time)
{
  for(int i=0;i<beep_time;i++)
  {
    tone(tonePin, 666);
    delay(200);
    noTone(tonePin);
    delay(10);
  }
}
