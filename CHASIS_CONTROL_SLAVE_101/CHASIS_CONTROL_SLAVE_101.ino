// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------------ SLAVE 101 ------------------------------------
//Libraries
#include <Wire.h>
#include <avr/sleep.h>

//Commands for I2C interface
#define DTM 99                          // 99 - остановить выполнение всех функций на устройстве на 1000 мс
#define EMP 100                         // 100 - НИЧЕГО, НУЛЬ
#define SLEEP 102                       // 102 - отправить контроллеры в сон SLEEP
#define WUP 103                         // 103 - разбудить контроллеры WAKE UP
#define TEST 104                        // 104 - тест всех датчиков и приводов
#define RST 109                         // 109 - комманда RESET
#define STP 110                         // 110 - СТОП
#define FWD 111                         // 111 - ВПЕРЕД
#define BWD 112                         // 112 - НАЗАД
#define SPU 115                         // 115 - УСКОРИТЬ
#define SDN 116                         // 116 - ЗАМЕДЛИТЬ
#define LFT 121                         // 121 - поворот НАЛЕВО
#define RGT 122                         // 122 - поворот НАПРАВО
#define TBK 123                         // 123 - РАЗВОРОТ на 180
#define TLT 131                         // 131 - ВКЛЮЧИТЬ СВЕТ
#define PLT 132                         // 132 - ВЫКЛЮЧИТЬ СВЕТ
#define SHB 133                         // 133 - сделать подсветку ЯРЧЕ
#define SHD 134                         // 134 - сделать подсветку ТУСКЛЕЕ
#define MGM 140                         // 140 - получить показания магнитометра
#define SUNON 141                       // 141 - режим СОЛНЕЧНОЙ БАТАРЕИ ВКЛЮЧЁН         глобальный
#define SUNOFF 142                      // 142 - режим СОЛНЕЧНОЙ БАТАРЕИ ВЫКЛЮЧЕН        глобальный 

//Error codes
#define OK 200
#define MEGNETOMETERDATAERROR 255

//System variables
#define MAGNETOMETR_HMC5883_MESASURING_COMMAND1 0x0B      // Tell the HMC5883 to Continuously Measure
#define MAGNETOMETR_HMC5883_MESASURING_COMMAND2 0x09      // Tell the HMC5883 to Continuously Measure
#define MAGNETOMETR_HMC5883_ADDRESS 0x0D                  //I2C Address for The HMC5883 magnetometer
#define MAGNETOMETER_REGISTER_1 0x01                      // Set the Register 1
#define MAGNETOMETER_REGISTER_2 0x1D                      // Set the Register 2
#define MAGNETOMETER_REGISTER_3 0x00                      // Set the Register 3
#define THIS_SLAVE_DEVICE_NUMBER 0x65                     // I2C-номер данного устройства
#define MASTER_DEVICE_SENSORS 0x64                        // I2C-номер устройства Sensors Shield (101)
#define SLAVE_DEVICE_102 0x66                             // I2C-номер устройства 102
#define DALAY_TIME 150                                    // время задержки по умолчанию 150мс
#define COMMAND_DELAY_TIME 1000                           // время задержки по комманде DTM ( 99 )
#define MAXIMAL_MOTOR_AMPERAGE 17                         // значение соответствует напряжению yV

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

//Used PINs
#define INTERRUPT_PIN 3                           // пин для отработки прерывания Выход из Сна
#define VOLTMETER_ONLEFT_MOTOR_SENSOR_PIN A0      // вольтметр питания левого двигателя
#define VOLTMETER_ONRIGHT_MOTOR_SENSOR_PIN A1     // вольтметр питания правого двигателя
#define SDA A4
#define SCL A5

//Global variables
byte mode = 0;                                    //последний режим 
unsigned long dTRotationAngleEstimate = 0;        // задержка времени в Actions()

class ChasisActions
{
  public:
    ChasisActions()
    {
       tankDirection = 0;
       tankSpeed = 160;
       lightBrightness = 0;
       engineTorqueRatio = 24;
    }
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
    volatile int tankSpeed;                                   // скорость (MIN = 0, MAX = 255)

  private:
    float GetRotationAngle();
    int lightBrightness;                       // сила подсветки
    int engineTorqueRatio;                     // разница передачи ШИМ сигнала на двигателя
    byte tankDirection;                        // напрвление (вперёд, назад) 
    int rotationAngle;                         // угол поворота, согласно показаниям магнитометра  
      
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

class TestClass
{
  public:
    TestClass();
    ~TestClass();
    int RunMagnetometerTest();
};

class Magnetometer
{
  public:
    Magnetometer()
    {
      axisXcurrent = 0;
      axisYcurrent = 0;
      axisZcurrent = 0;
      horizontalAngle = 0;
      compas = 0;
      verticalAngle = 0;

      xvarVolt = 0.9;  
      xvarProcess = 0.05; 
      xPc = 0.0;
      xG = 0.0;
      xP = 1.0;
      xXp = 0.0;
      xZp = 0.0;
      xXe = 0.0;

      yvarVolt = 0.5;  
      yvarProcess = 0.05; 
      yPc = 0.0;
      yG = 0.0;
      yP = 1.0;
      yXp = 0.0;
      yZp = 0.0;
      yXe = 0.0;

      zvarVolt = 0.9;  
      zvarProcess = 0.05; 
      zPc = 0.0;
      zG = 0.0;
      zP = 1.0;
      zXp = 0.0;
      zZp = 0.0;
      zXe = 0.0;
    }
    
    ~Magnetometer();
    void GetRotationAngles();
    float horizontalAngle, compas, verticalAngle;                                       // углы поворота по осям X, Y, Z   
    
  private:  
    void GetMagnetometerData();
    float filterX(float val);
    float filterY(float val);
    float filterZ(float val);
    float axisXcurrent,axisYcurrent, axisZcurrent;                      //текущее значение напряженности магнитного поля по осям  
    float xvarVolt, xvarProcess, xPc, xG, xP, xXp, xZp, xXe;            //для фильтра Калмана оси X
    float yvarVolt ,yvarProcess, yPc, yG, yP, yXp, yZp, yXe;            //для фильтра Калмана оси Y
    float zvarVolt ,zvarProcess, zPc, zG, zP, zXp, zZp, zXe;            //для фильтра Калмана оси Z
};

class Command
{
  public:
    Command();
    ~Command();
    void SendErrorToMaster();
    void SendOkToMaster();

  private:
    void SendCommandToMaster(byte command);
    void SendCommandTo102(byte command);
};

ChasisActions action;
Motor motor;
TestClass tests;
Magnetometer mag;
Command cmd;

void setup()
{
  Serial.begin(9600);
  Wire.begin(THIS_SLAVE_DEVICE_NUMBER);
  Wire.onReceive(OnReceiveEventHandler);
  pinMode(VOLTMETER_ONLEFT_MOTOR_SENSOR_PIN, INPUT);
  pinMode(VOLTMETER_ONRIGHT_MOTOR_SENSOR_PIN, INPUT);
  dTRotationAngleEstimate = millis();
}

void OnReceiveEventHandler(int bytes)   //получение команды через I2C
{
  //Serial.print("Available to read: ");Serial.println(Wire.available());
  if (Wire.available() > 0 && Wire.available() < 2)
  {
    byte in_data = Wire.read();
    interrupts();
    ChooseAction(in_data);
  }
}

void SelfTestStart()
{  
  
  if(tests.RunMagnetometerTest() == OK)
  {
    cmd.SendOkToMaster();
  }
  else
  {
    cmd.SendErrorToMaster();
  }  
}

void loop()
{}

void WakeUpNow()                      //обработка прерывания на порте D3(пробуждение контроллера)
{}

void ChooseAction(byte cmd)           // выбор действия в зависимости от полученой команды
{
  //Serial.println(cmd);
  dTRotationAngleEstimate = millis();
  switch (cmd)
  {
    case DTM:
      DelayController();
      break;
    case EMP:
      mode = EMP;
      break;
    case RST:
      action.ActionResetTankMode();
      break;
    case SLEEP:
      action.ActionResetTankMode();
      ActionSetControlerToSleep();
      break;
    case TEST:
      action.ActionStopTank();
      SelfTestStart();
      break;
    case STP:
      action.ActionStopTank();
      break;
    case FWD:
      action.ActionMoveTankForward();
      break;
    case BWD:
      action.ActionMoveTankBackward();
      break;
    case LFT:
      action.ActionTurnTankLeft();
      break;
    case RGT:
      action.ActionTurnTankRight();
      break;
    case TBK:
      action.ActionTurnTankBack();
      break;
    case SPU:
      action.ActionSpeedUpTank();
      break;
    case SDN:
      action.ActionSlowDownTank();
      break;
    case TLT:
      action.ActionTurnOnTheLight();
      break;
    case PLT:
      action.ActionPutOutTheLight();
      break;
    case SHB:
      action.ActionShineBrighter();
      break;
    case SHD:
      action.ActionShineDimmer();
      break;
    case MGM:    
      mag.GetRotationAngles();
      break;
    default:
      break;
  }
  cmd = EMP;
  GetSpeedDependence();
}

void GetSpeedDependence()
{
  float amperageLeftMotor = GetMotorVoltage(LEFT_MOTOR);
  float amperageRightMotor = GetMotorVoltage(RIGHT_MOTOR);

  // Serial.print("amperageLeftMotor. "); Serial.println(amperageLeftMotor);
  //Serial.print("amperageRightMotor. "); Serial.println(amperageRightMotor);

  if (amperageLeftMotor >= MAXIMAL_MOTOR_AMPERAGE &&
      amperageRightMotor >= MAXIMAL_MOTOR_AMPERAGE &&
      action.tankSpeed < 250)
  {
    action.tankSpeed += 10;
  }
  if (amperageLeftMotor < MAXIMAL_MOTOR_AMPERAGE &&
      amperageRightMotor < MAXIMAL_MOTOR_AMPERAGE)
  {
    action.tankSpeed = 160;
  }
}

float GetMotorVoltage(byte motorNumber)
{
  float data = 0;
  byte avarage = 50;
  switch (motorNumber)
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
  attachInterrupt(1, WakeUpNow, LOW);
  sleep_mode();
  //отслюда после пробуждения
  sleep_disable();
  detachInterrupt(1);
  interrupts();
  mode = EMP;
}

void DelayController()
{  
  delay(COMMAND_DELAY_TIME);  
}
