// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------------ SLAVE 101 ------------------------------------
#include <Wire.h>
#include <avr/sleep.h>

#define EMP 0                         // 0 - НИЧЕГО, НУЛЬ
#define SLEEP 2                       // 2 - отправить контроллеры в сон SLEEP
#define WUP 3                         // 3 - разбудить контроллеры WAKE UP
#define RST 9                         // 9 - комманда RESET
#define STP 10                        // 10 - СТОП
#define FWD 11                        // 11 - ВПЕРЕД
#define BWD 12                        // 12 - НАЗАД
#define SPU 15                        // 15 - УСКОРИТЬ
#define SDN 16                        // 16 - ЗАМЕДЛИТЬ
#define LFT 21                        // 21 - поворот НАЛЕВО
#define RGT 22                        // 22 - поворот НАПРАВО
#define TBK 23                        // 23 - РАЗВОРОТ на 180
#define TLT 31                        // 31 - ВКЛЮЧИТЬ СВЕТ
#define PLT 32                        // 32 - ВЫКЛЮЧИТЬ СВЕТ
#define SHB 33                        // 33 - сделать подсветку ЯРЧЕ
#define SHD 34                        // 34 - сделать подсветку ТУСКЛЕЕ
#define SUNON 41                      // 41 - режим СОЛНЕЧНОЙ БАТАРЕИ ВКЛЮЧЁН
#define SUNOFF 42                     // 42 - режим СОЛНЕЧНОЙ БАТАРЕИ ВЫКЛЮЧЕН 

#define THIS_SLAVE_DEVICE_NUMBER 101  // I2C-номер данного устройства
#define DALAY_TIME 150                // время задержки по умолчанию 500мс

#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5

#define motorFORWARD 1
#define motorBACKWARD 2
#define motorRELEASE 3

#define INTERRUPT_PIN 3

byte mode = 0;                        //последний режим
int tankSpeed = 0;                    //для проверки скорости танка
byte tankDirection = 0;               //для проверки направления движения
int lightBrightness = 0;              // сила подсветки
int sunBrightness = 0;                // освещенность солнцем
unsigned long dTtemp = 0;             // выдержка в Actions()

void setup()
{
  Serial.begin(9600);
  Wire.begin(THIS_SLAVE_DEVICE_NUMBER);
  Wire.onReceive(OnReceiveEventHandler);
}

void(* resetFunc) (void) = 0;

void OnReceiveEventHandler(int bytes)   //получение команды через I2C
{
  byte in_data = Wire.read();
  interrupts();
  ChooseAction(in_data);
  Serial.println(in_data);
}

void loop()
{}

void WakeUpNow()                      //обработка прерывания на порте D3
{}

void ChooseAction(byte in_data)       // выбор действия в зависимости от полученой команды
{
  dTtemp = millis();
  Serial.println(in_data);
  switch (in_data)
  {
    case EMP:                         
      mode = EMP;
      break;  
    case RST:                               
      ActionResetTankMode();
      resetFunc(); 
      break;  
    case SLEEP:
      ActionSetControlerToSleep();    
      break;      
    case STP:                              
      ActionStopTank();
      break;
    case FWD:                         
      ActionMoveTankForward(170);
      break;
    case BWD:                             
      ActionMoveTankBackward(170);
      break;
    case LFT:                              
      ActionTurnTankLeft();
      break;
    case RGT:                         
      ActionTurnTankRight();
      break;
    case TBK:                         
      ActionTurnTankBack();
      break;
    case SPU:                         
      ActionSpeedUpTank();
      break;
    case SDN:                         
      ActionSlowDownTank();
      break;
    case TLT:                         
      ActionTurnOnTheLight();
      break;
    case PLT:                         
      ActionPutOutTheLight();
      break;
    case SHB:                         
      ActionShineBrighter();
      break;
    case SHD:                         
      ActionShineDimmer();
      break;    
    default:
      break;      
  }  
  in_data = EMP;
}

void ActionResetTankMode()                        //RESET
{
  mode = EMP;
  ActionStopTank();
  ActionPutOutTheLight();
}

void ActionSetControlerToSleep()                  // отправить устройство в сон
{
  mode = SLEEP;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   
  sleep_enable();
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(1,WakeUpNow, LOW);
  sleep_mode();
                //отслюда после пробуждения
  sleep_disable();
  detachInterrupt(1);
  interrupts();
  mode = EMP;
}

void ActionMoveTankForward(byte motorSpeed)       //вперёд
{
  tankDirection = motorFORWARD;
  tankSpeed = motorSpeed;
  chooseMotor(1, tankDirection, motorSpeed);
  chooseMotor(2, tankDirection, motorSpeed-52);  
  mode = FWD;
}

void ActionMoveTankBackward(byte motorSpeed)        //назад 
{
  tankDirection = motorBACKWARD;
  tankSpeed = motorSpeed;
  chooseMotor(1, tankDirection, motorSpeed);
  chooseMotor(2, tankDirection, motorSpeed-52);
  mode = BWD;
}

void ActionStopTank()                               //стоп 
{
  chooseMotor(1, motorRELEASE, 0);
  chooseMotor(2, motorRELEASE, 0);
  tankSpeed = 0;
  tankDirection = 0;
  mode = STP;
}

void ActionTurnTankLeft()                           //поворот влево 
{
  while(millis() - dTtemp < DALAY_TIME)
  {
    chooseMotor(1, motorBACKWARD, 200);
    chooseMotor(2, motorFORWARD, 200);
  }  
  if(mode == FWD)
  {
    ActionMoveTankForward(150);
  }
  else if(mode == BWD)
  {
    ActionMoveTankBackward(150);
  }
  else 
  {
    ActionStopTank();
  }  
}

void ActionTurnTankRight()                          //поворот вправо
{
  while(millis() - dTtemp < DALAY_TIME)
  {
    chooseMotor(1, motorFORWARD, 200);
    chooseMotor(2, motorBACKWARD, 200);
  }
  if(mode == FWD)
  {
    ActionMoveTankForward(150);
  }
  else if(mode == BWD)
  {
    ActionMoveTankBackward(150);
  }
  else 
  { 
    ActionStopTank();
  } 
}

void ActionTurnTankBack()                           //разворот на 180
{
  while(millis() - dTtemp < DALAY_TIME*2)
  {
    chooseMotor(1, motorFORWARD, 200);
    chooseMotor(2, motorBACKWARD, 200);
  }  
  if(mode == FWD)
  {
    ActionMoveTankForward(150);
  }
  else if(mode == BWD)
  {
    ActionMoveTankBackward(150);
  }
  else 
  {
    ActionStopTank(); 
  } 
}

void ActionSpeedUpTank()                            //ускорить
{
  tankSpeed += 10;
  if (tankDirection !=0 && tankSpeed > 0 && tankSpeed < 255)
  {
    if (tankDirection == motorFORWARD)
    {
      ActionMoveTankForward(tankSpeed);
    }
    else
    {
      ActionMoveTankBackward(tankSpeed);
    }
  }
}

void ActionSlowDownTank()                           //замедлить
{
  tankSpeed -= 10;
  if (tankDirection !=0 && tankSpeed > 0)
  {
    if (tankDirection == motorFORWARD)
    {
      ActionMoveTankForward(tankSpeed);
    }
    else
    {
      ActionMoveTankBackward(tankSpeed);
    }
  }
}

void ActionTurnOnTheLight()                         //включить свет
{
  if(lightBrightness == 0)
  {
    lightBrightness = 10;  
    chooseMotor(3, motorFORWARD, lightBrightness);    
    mode = TLT;
  }
}

void ActionPutOutTheLight()                         //выключить свет
{
  if(lightBrightness > 0)
  {
    lightBrightness = 0;
    chooseMotor(3,motorRELEASE,0);    
    mode = PLT;
  }
}

void ActionShineBrighter()                          //усилить яркость
{
  lightBrightness += 10;
  if(lightBrightness > 50)lightBrightness = 50;
  chooseMotor(3, motorFORWARD, lightBrightness);
}

void ActionShineDimmer()                          //уменьшить яркость
{
  lightBrightness -=10;
  if(lightBrightness <= 0)
  {
    ActionPutOutTheLight();
  }
  else
  {
    chooseMotor(3, motorFORWARD, lightBrightness);
  }
}

void chooseMotor(int motorNumber, int command, int motorSpeed)    // выбор двигетеля
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

void motor_output (int output, int high_low, int motorSpeed)
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

void shiftWrite(int output, int high_low)
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
