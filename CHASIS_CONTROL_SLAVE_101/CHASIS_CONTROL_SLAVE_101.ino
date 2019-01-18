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
#define DALAY_TIME 150                // время задержки по умолчанию 150мс
#define MINIMAL_MOTOR_AMPERAGE 14     // значение соответствует напряжению xV
#define MAXIMAL_MOTOR_AMPERAGE 16     // значение соответствует напряжению yV
#define LEFT_MOTOR 1                  // двигатель №1 - левый
#define RIGHT_MOTOR 2                 // двигатель №2 - правый
#define LED 3                         // двигатель №3 - светодиод

#define MOTORLATCH 12                 // пины для корректной работы шилда Двигателей
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

#define motorFORWARD 1                            // режим ВПЕРЁД
#define motorBACKWARD 2                           // режим НАЗАД
#define motorRELEASE 3                            // освободить Двигатель

#define INTERRUPT_PIN 3                           // пин для отработки прерывания Выход из Сна
#define VOLTMETER_ONLEFT_MOTOR_SENSOR_PIN A0      // вольтметр питания левого двигателя
#define VOLTMETER_ONRIGHT_MOTOR_SENSOR_PIN A1     // вольтметр питания правого двигателя

byte mode = 0;                                    //последний режим
int tankSpeed = 180;                                //для проверки скорости танка
byte tankDirection = 0;                           //для проверки направления движения
int lightBrightness = 0;                          // сила подсветки
int sunBrightness = 0;                            // освещенность солнцем
unsigned long dTtemp = 0;                         // задержка времени в Actions()
unsigned long dTloopGeneral = 0;                  // задержка времени в loop()
byte engineTorqueRatio = 40;                      // разница передачи ШИМ сигнала на двигателя

class ChasisActions
{
  public:
    ChasisActions();
    ~ChasisActions();
    void ActionResetTankMode();                               //RESET
    void ActionMoveTankForward();                             //вперёд
    void ActionMoveTankBackward();                            //назад
    void ActionStopTank();                                    //стоп 
    void ActionTurnTankLeft();                                //поворот влево 
    void ActionTurnTankRight();                               //поворот вправо
    void ActionTurnTankBack();                                //разворот на 180
    void ActionSpeedUpTank();                                 //ускорить
    void ActionSlowDownTank();                                //замедлить
    void ActionTurnOnTheLight();                              //включить свет
    void ActionPutOutTheLight();                              //выключить свет
    void ActionShineBrighter();                               //усилить яркость
    void ActionShineDimmer();                                 //уменьшить яркость    
};

class Motor
{
  public:
    Motor();
    ~Motor();
    void chooseMotor(int motorNumber, int command, int motorSpeed);    // выбор двигетеля

  private:
    void motor_output (int output, int high_low, int motorSpeed);
    void shiftWrite(int output, int high_low);
};

ChasisActions *action;
Motor *motor;

void setup()
{
  Wire.begin(THIS_SLAVE_DEVICE_NUMBER);
  Wire.onReceive(OnReceiveEventHandler);
  pinMode(VOLTMETER_ONLEFT_MOTOR_SENSOR_PIN, INPUT);
  pinMode(VOLTMETER_ONRIGHT_MOTOR_SENSOR_PIN, INPUT);
  Serial.begin(9600);
}

void(*resetFunc) (void) = 0;

void OnReceiveEventHandler(int bytes)   //получение команды через I2C
{
  byte in_data = Wire.read();
  interrupts();
  ChooseAction(in_data);
  Serial.println(in_data);
}

void loop()
{
  if(millis() - dTloopGeneral > 1000)
  {
    dTloopGeneral = millis();
    GetSpeedDependence();
  }
}

void WakeUpNow()                      //обработка прерывания на порте D3
{}

void ChooseAction(byte in_data)       // выбор действия в зависимости от полученой команды
{
  dTtemp = millis();
  switch (in_data)
  {
    case EMP:                         
      mode = EMP;
      break;  
    case RST:                               
      action->ActionResetTankMode();
      resetFunc(); 
      break;  
    case SLEEP:
      ActionSetControlerToSleep();    
      break;      
    case STP:                              
      action->ActionStopTank();
      break;
    case FWD:                         
      action->ActionMoveTankForward();
      break;
    case BWD:                             
      action->ActionMoveTankBackward();
      break;
    case LFT:                              
      action->ActionTurnTankLeft();
      break;
    case RGT:                         
      action->ActionTurnTankRight();
      break;
    case TBK:                         
      action->ActionTurnTankBack();
      break;
    case SPU:                         
      action->ActionSpeedUpTank();
      break;
    case SDN:                         
      action->ActionSlowDownTank();
      break;
    case TLT:                         
      action->ActionTurnOnTheLight();
      break;
    case PLT:                         
      action->ActionPutOutTheLight();
      break;
    case SHB:                         
      action->ActionShineBrighter();
      break;
    case SHD:                         
      action->ActionShineDimmer();
      break;    
    default:
      break;      
  }  
  in_data = EMP;
}

void GetSpeedDependence()
{
    float amperageLeftMotor = GetMotorVoltage(LEFT_MOTOR);
    float amperageRightMotor = GetMotorVoltage(RIGHT_MOTOR);

    if(amperageLeftMotor < MINIMAL_MOTOR_AMPERAGE &&
        amperageRightMotor < MINIMAL_MOTOR_AMPERAGE)
    {                                   //логика такова, если этот показатель уменьшается, значит нагрузка на двигатель низкая, мощность можно снизить в целях экономии энергии
      action->ActionSlowDownTank();
    }
    else if(amperageLeftMotor > MAXIMAL_MOTOR_AMPERAGE &&
              amperageRightMotor > MAXIMAL_MOTOR_AMPERAGE)
    {                                   //логика такова, если этот показатель повышается, значит двигатель сильно нагружен, нужно подать большую мощность
      action->ActionSpeedUpTank();
    }
}

float GetMotorVoltage(byte motorNumber)
{
  float data = 0;
  byte avarage = 100;
  switch(motorNumber)
  {
    case 1:     
      for (byte i = 0; i < avarage; i ++)
      {
        data += analogRead(VOLTMETER_ONLEFT_MOTOR_SENSOR_PIN) * 5.0 / 102.4;
      }    
      break;
    case 2:
      for (byte i = 0; i < avarage; i ++)
      {
        data += analogRead(VOLTMETER_ONRIGHT_MOTOR_SENSOR_PIN) * 5.0 / 102.4;
      }
      break;
  }
  return (float)(data / avarage);
}

void ActionSetControlerToSleep()                  // отправить устройство в сон
{  
  action->ActionStopTank();
  action->ActionPutOutTheLight();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   
  sleep_enable();
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(1,WakeUpNow, LOW);
  mode = SLEEP;
  sleep_mode();
                //отслюда после пробуждения
  sleep_disable();
  detachInterrupt(1);
  interrupts();
  mode = EMP;
}