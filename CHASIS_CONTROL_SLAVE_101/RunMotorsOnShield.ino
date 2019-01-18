// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------------ SLAVE 101 ------------------------------------

void Motor :: chooseMotor(int motorNumber, int command, int motorSpeed)    // выбор двигетеля
{
  int motorA, motorB;

  if (motorNumber >= 1 && motorNumber <= 4)
  {
    switch (motorNumber)
    {
      case 1:
        motorA   = MOTOR1_A;
        motorB   = MOTOR1_B;
        break;
      case 2:
        motorA   = MOTOR2_A;
        motorB   = MOTOR2_B;
        break;
      case 3:
        motorA   = MOTOR3_A;
        motorB   = MOTOR3_B;
        break;
      case 4:
        motorA   = MOTOR4_A;
        motorB   = MOTOR4_B;
        break;
      default:
        break;
    }

    switch (command)
    {
      case motorFORWARD:
        motor_output (motorA, HIGH, motorSpeed);
        motor_output (motorB, LOW, -1);
        break;
      case motorBACKWARD:
        motor_output (motorA, LOW, motorSpeed);
        motor_output (motorB, HIGH, -1);
        break;
      case motorRELEASE:
        motor_output (motorA, LOW, 0);
        motor_output (motorB, LOW, -1);
        break;
      default:
        break;
    }
  }
}

void Motor :: motor_output (int output, int high_low, int motorSpeed)
{
  int motorPWM;

  switch (output)
  {
    case MOTOR1_A:
    case MOTOR1_B:
      motorPWM = MOTOR1_PWM;
      break;
    case MOTOR2_A:
    case MOTOR2_B:
      motorPWM = MOTOR2_PWM;
      break;
    case MOTOR3_A:
    case MOTOR3_B:
      motorPWM = MOTOR3_PWM;
      break;
    case MOTOR4_A:
    case MOTOR4_B:
      motorPWM = MOTOR4_PWM;
      break;
    default:
      motorSpeed = -3333;
      break;
  }

  if (motorSpeed != -3333)
  {
    shiftWrite(output, high_low);
    if (motorSpeed >= 0 && motorSpeed <= 255)
    {
      analogWrite(motorPWM, motorSpeed);
    }
  }
}

void Motor :: shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;
  if (!shift_register_initialized)
  {
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    digitalWrite(MOTORENABLE, LOW);
    latch_copy = 0;
    shift_register_initialized = true;
  }
  bitWrite(latch_copy, output, high_low);
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);
  digitalWrite(MOTORLATCH, LOW);
}
