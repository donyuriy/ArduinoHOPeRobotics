// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------------ SLAVE 101 ------------------------------------
#include <Wire.h>
#include <avr/sleep.h>

#define EMP 100                         // 100 - НИЧЕГО, НУЛЬ
#define SLEEP 102                       // 102 - отправить контроллеры в сон SLEEP
#define WUP 103                         // 103 - разбудить контроллеры WAKE UP
#define RST 109                         // 109 - комманда RESET
#define STP 110                        // 110 - СТОП
#define FWD 111                        // 111 - ВПЕРЕД
#define BWD 112                        // 112 - НАЗАД
#define SPU 115                        // 115 - УСКОРИТЬ
#define SDN 116                        // 116 - ЗАМЕДЛИТЬ
#define LFT 121                        // 121 - поворот НАЛЕВО
#define RGT 122                        // 122 - поворот НАПРАВО
#define TBK 123                        // 123 - РАЗВОРОТ на 180
#define TLT 131                        // 131 - ВКЛЮЧИТЬ СВЕТ
#define PLT 132                        // 132 - ВЫКЛЮЧИТЬ СВЕТ
#define SHB 133                        // 133 - сделать подсветку ЯРЧЕ
#define SHD 134                        // 134 - сделать подсветку ТУСКЛЕЕ

#define THIS_SLAVE_DEVICE_NUMBER 0x65  // I2C-номер данного устройства
#define DALAY_TIME 150                // время задержки по умолчанию 150мс
#define MINIMAL_MOTOR_AMPERAGE 10     // значение соответствует напряжению xV
#define MAXIMAL_MOTOR_AMPERAGE 13     // значение соответствует напряжению yV
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
#define SDA A4
#define SCL A5

byte mode = 0;                                    //последний режим
byte tankDirection = 0;                           //для проверки направления движения
int lightBrightness = 0;                          // сила подсветки
int sunBrightness = 0;                            // освещенность солнцем
unsigned long dTtemp = 0;                         // задержка времени в Actions()
unsigned long dTloopGeneral = 0;                  // задержка времени в loop()
byte engineTorqueRatio = 40;                      // разница передачи ШИМ сигнала на двигателя
volatile int tankSpeed;

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
  //Serial.begin(9600);
  Wire.begin(THIS_SLAVE_DEVICE_NUMBER);
  Wire.onReceive(OnReceiveEventHandler);
  pinMode(VOLTMETER_ONLEFT_MOTOR_SENSOR_PIN, INPUT);
  pinMode(VOLTMETER_ONRIGHT_MOTOR_SENSOR_PIN, INPUT); 
  tankSpeed = 180;
}

void OnReceiveEventHandler(int bytes)   //получение команды через I2C
{
  byte in_data = Wire.read();
  interrupts();
  ChooseAction(in_data);  
}

void loop()
{ 
  if(millis() - dTloopGeneral > 2500)
  {
    dTloopGeneral = millis();
    GetSpeedDependence();
  }
}

void WakeUpNow()                      //обработка прерывания на порте D3
{}

void ChooseAction(byte cmd)           // выбор действия в зависимости от полученой команды
{
  //Serial.println(cmd);
  dTtemp = millis();
  switch (cmd)
  {
    case EMP:                         
      mode = EMP;
      break;  
    case RST:                               
      action->ActionResetTankMode();
      break;  
    case SLEEP:
      action->ActionResetTankMode();
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
  cmd = EMP;
}

void GetSpeedDependence()
{
    float amperageLeftMotor = GetMotorVoltage(LEFT_MOTOR);
    float amperageRightMotor = GetMotorVoltage(RIGHT_MOTOR);
    
    //Serial.print("amperageLeftMotor-> "); Serial.println(amperageLeftMotor);
    //Serial.print("amperageRightMotor-> "); Serial.println(amperageRightMotor);
    
    if(amperageLeftMotor > MAXIMAL_MOTOR_AMPERAGE &&
              amperageRightMotor > MAXIMAL_MOTOR_AMPERAGE)
    {  
      action->ActionStopTank();                        
      action->ActionMoveTankBackward();
      delay(5);  
      action->ActionTurnTankLeft();
      delay(5);  
      action->ActionMoveTankForward();
    }
}

float GetMotorVoltage(byte motorNumber)
{
  float data = 0;
  byte avarage = 50;
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
