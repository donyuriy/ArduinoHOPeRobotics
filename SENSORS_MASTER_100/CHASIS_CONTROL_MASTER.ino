// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------- MASTER ----------------------------------------

#include <Servo.h>
#include <Wire.h>
#include <avr/sleep.h>

#define EMP 100                         // 100 - НИЧЕГО, НУЛЬ
#define SLEEP 102                       // 102 - отправить контроллеры в сон SLEEP
#define WUP 103                         // 103 - разбудить контроллеры WAKE UP
#define TEST 104                        // 104 - тест всех датчиков и приводов
#define RST 109                         // 19 - комманда RESET
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
#define SUNON 141                       // 141 - режим СОЛНЕЧНОЙ БАТАРЕИ ВКЛЮЧЁН         глобальный
#define SUNOFF 142                      // 142 - режим СОЛНЕЧНОЙ БАТАРЕИ ВЫКЛЮЧЕН        глобальный 

//Error codes
#define OK 200
#define LEFTUSSENSORERROR 401
#define RIGHTUSSENSORERROR 402
#define CENTRALUSSENSORERROR 403
#define SERVOUSSENSORERROR 411
#define SERVOSOLARHORIZONTALERROR 412
#define SERVOSOLARVERTICALERROR 413
#define PHOTOSENSORSOLAR1ERROR 421
#define PHOTOSENSORSOLAR2ERROR 422
#define PHOTOSENSORSOLAR3ERROR 423


#define SLAVE_DEVICE_CHASIS 0x65
#define SLAVE_DEVICE_CAMERA 0x66
#define LOWEST_BATTERY_CHARGE 2.51                        // значение соответствует напряжению 2.93 вольта 
#define LOW_BATTERY_CHARGE 2.92                           // значение соответствует напряжению 3.43 вольта  (остаток 10% )
#define HIGH_BATTERY_CHARGE 3.3                           // значение соответствует напряжению 4 вольта
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_SOLAR_BATTERY 200    // значение освещенности для ВКЛЮЧЕНИЯ Солнечной Батареи
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_AWAKE 400            // значение освещенности для ПРОСНУТЬСЯ
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT 600     // значение освещенности для ВКЛЮЧЕНИЯ ОСВЕЩЕНИЯ
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP 800            // значение освещенности для перехода в РЕЖИМ СНА
#define MIN_SOLAR_VERTICAL_ANGLE 110                      // минимальный угол поворота по вертикали ^
#define MAX_SOLAR_VERTICAL_ANGLE 160                      // максимальный угол поворота по вертикали ^
#define MIN_SOLAR_HORIZONTAL_ANGLE 5                      // минимальный угол поворота Солнечной Панели по горизонтали <- |
#define MAX_SOLAR_HORIZONTAL_ANGLE 175                    // максимальный угол поворота Солнечной Панели по горизонтали | ->

#define ERRORLED 0                                  // показатель ошибки при самотестировании
#define INTERRUPT_0_PIN 2                           // порт для обработки прерываний D2 (interrupt #0)
#define INTERRUPT_1_PIN 3                           // порт для обработки прерываний D3 (interrupt #1)
#define ULTRASOUND_LEFT_SENSOR_TRIGGER_PIN 4        // УЗ-ЛЕВЫЙ сенсор расстояния передатчик
#define ULTRASOUND_LEFT_SENSOR_ECHO_PIN 5           // УЗ-ЛЕВЫЙ сенсор расстояния приёмник
#define ULTRASOUND_RIGHT_SENSOR_TRIGGER_PIN 6       // УЗ-ПРАВЫЙ сенсор расстояния передатчик
#define ULTRASOUND_RIGHT_SENSOR_ECHO_PIN 7          // УЗ-ПРАВЫЙ сенсор расстояния приёмник
#define SERVO_SUN_BATTERY_MOTOR_1 8                 // сервопривод управления Солнечной панелью (горизонт)
#define SERVO_SUN_BATTERY_MOTOR_2 9                 // сервопривод управления Солнечной панелью (вертикаль)
#define SERVO_ULTRASOUND_SENSOR_PIN 10              // сервопривод управления УЗ-сенсором расстояния
#define ULTRASOUND_CENTRAL_SENSOR_TRIGGER_PIN 11    // УЗ-ЦЕНТРАЛЬНЫЙ сенсор расстояния передатчик
#define ULTRASOUND_CENTRAL_SENSOR_ECHO_PIN 12       // УЗ-ЦЕНТРАЛЬНЫЙ сенсор расстояния приёмник
#define OUTPUT_WAKEUP_INTERRUPT_PIN 13              // для отправки цифрового сигнала для прерывания пробуждения вспомогательных шилдов

#define VOLTMETER_SENSOR_PIN A0                     // вольтметр батареи
#define SOLAR_SENSOR_PIN_2 A2                       // сенсор освещенности 1
#define SOLAR_SENSOR_PIN_1 A3                       // сенсор освещенности 2
#define SOLAR_SENSOR_PIN_3 A1                       // сенсор освещенности 3

unsigned long dTforUSsensor = 0;                   // задержка времени для предотврашения застреваня в узких для поворота местах
unsigned long dTlight = 0;                         // задержка времени . ВКЛ/ВЫКЛ освещение
unsigned long dTvoltage = 0;                       // задержка времени . проверка заряда батареи
unsigned long dTsolar = 0;                         // задержка времени . период перекалибровки положения солнечной панели
unsigned long timeToSleep = 0;                     // оживание до сасыпания
volatile unsigned long interruptorTime = 0;        // для срабатывание функции прерывания на d2 (OnSoundInterrupt)

byte actionsCounter = 0;                           // количество повторяющихся поворотов за последние N секунд
byte globalMode = 0;                               // последний установленный режим (см. пометку "глобальный")
byte extraMode = 0;                                // дополнительный режим
byte verticalSunBattery_angle;                     // положение вертикального двигателя солнечной батареи
byte horizontalSunBattery_angle;                   // положение горизонтального двигателя солнечной батареи
byte angleDifference = 2;                          // разница показаний сервоприводов для операций Солнечной батареи
int errorLevel = 0;                                 // ошибка в процессе тестирования
byte photosensorDefference = 2;                    // разница показаний фотосенсоров для операций Солнечной батареи
volatile bool enginesEnabled = true;

class Command
{
  public:
  Command();
  ~Command();
  void ResetCmd();
  void MoveForwardCmd();
  void MoveBackCmd();
  void TurnRightCmd();
  void TurnLeftCmd();
  void TurnBackCmd();
  void StopTankCmd();
  void SpeedUpCmd();
  void SlowDownCmd();
  void TurnOnTheLightCmd();
  void PutOutTheLightCmd();
  void ShineBrighterCmd();
  void ShineDimmerCmd();
  void SetSleepModeCmd();
  void RunTest();

  private:
  void SendCommandToChasis(byte command);
};

class BatteryClass
{
  public:
  BatteryClass();
  ~BatteryClass();
  float GetBattaryVoltage();
  bool IsBatteryPowerNormal();
  void CheckBatteryVoltage();
  void ActionSolarBatteryOn();
  void SetUpSolarBattery(Servo servoSunBatteryVertical, Servo servoSunBatteryHorizontal);
  void ActionSolarBatteryOff();
  float GetPhotoSensorData(byte sensorID);
  void RunServos(byte ServoStartAngle, byte ServoFinishAngle, Servo servo);
  
  private:  
  void SearchVerticalSolar();
  void SearchHorizontalSolar();  
  void SolarSearchingInMotion();  
};

BatteryClass bc;
Command sendCommand;

Servo servoUltrasoundSensor;
Servo servoSunBatteryVertical;
Servo servoSunBatteryHorizontal;

void setup()
{
  //Serial.begin(9600);
  Wire.begin();
  pinMode(ERRORLED, OUTPUT);
  pinMode(ULTRASOUND_CENTRAL_SENSOR_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASOUND_CENTRAL_SENSOR_ECHO_PIN, INPUT);
  pinMode(ULTRASOUND_LEFT_SENSOR_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASOUND_LEFT_SENSOR_ECHO_PIN, INPUT);
  pinMode(ULTRASOUND_RIGHT_SENSOR_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASOUND_RIGHT_SENSOR_ECHO_PIN, INPUT);    
  pinMode(OUTPUT_WAKEUP_INTERRUPT_PIN, OUTPUT);  
  pinMode(SOLAR_SENSOR_PIN_1, INPUT);
  pinMode(SOLAR_SENSOR_PIN_2, INPUT);
  pinMode(SOLAR_SENSOR_PIN_3, INPUT);
  pinMode(VOLTMETER_SENSOR_PIN, INPUT);
  pinMode(INTERRUPT_1_PIN, INPUT);
  servoUltrasoundSensor.attach(SERVO_ULTRASOUND_SENSOR_PIN);
  servoSunBatteryVertical.attach(SERVO_SUN_BATTERY_MOTOR_1);
  servoSunBatteryHorizontal.attach(SERVO_SUN_BATTERY_MOTOR_2);
  //attachInterrupt(0, OnSoundInterrupt, CHANGE); 
  delay(100);
  errorLevel = OK;
  RunSelfTest();
  OnStart();
  interruptorTime = millis();
}

void OnStart()
{
  sendCommand.ResetCmd();   
  bc.CheckBatteryVoltage();
}

void loop()
{ 
  if(errorLevel == OK)
  {
    if(globalMode != SUNON && enginesEnabled)
    {      
      CheckForObstackles();
    }
    
    if (millis() - dTlight > 2000 && enginesEnabled)
    {
      dTlight = millis();
      TurnOnOffLight();     
    }
  
    if (millis() - dTvoltage > 10000)
    {
      dTvoltage = millis();
      bc.CheckBatteryVoltage();
  
      
      if(globalMode == SUNON)
      {
        Flasher(1);
        dTsolar = millis();
        bc.ActionSolarBatteryOn();
      }
      
      if (bc.GetPhotoSensorData(1) > MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP &&
          bc.GetPhotoSensorData(2) > MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP &&
          bc.GetPhotoSensorData(3) > MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP &&
          millis() - timeToSleep > 300000)
      {
        SleepNow();
      }
      else
      {
        timeToSleep = millis();
      }
    }  
  } 
  else
  {
     Flasher(errorLevel);
  }
}

void RunSelfTest()
{
  if(errorLevel == 0 || errorLevel == OK)
  {
    errorLevel = SelfTestStart();
  }
  //Serial.print("Error on test: ");Serial.println(errorLevel);
  if(errorLevel != OK)
  {
    switch(errorLevel)
    {
      case LEFTUSSENSORERROR:
        Flasher(3);
        break;
      case RIGHTUSSENSORERROR:
        Flasher(3);
        break;
      case CENTRALUSSENSORERROR:
        Flasher(3);
        break;
      case SERVOUSSENSORERROR:
        Flasher(4);
        break;
      case SERVOSOLARHORIZONTALERROR:
        Flasher(5);
        break;
      case SERVOSOLARVERTICALERROR:
        Flasher(5);
        break;
      case PHOTOSENSORSOLAR1ERROR:
        Flasher(7);
        break;
      case PHOTOSENSORSOLAR2ERROR:
        Flasher(7);
        break;
      case PHOTOSENSORSOLAR3ERROR:
        Flasher(7);
        break;
      default:
        break;        
    }
  }
  else
  {
    digitalWrite(ERRORLED, LOW);
  }
}

void Flasher(byte count)
{
  for(byte i = 0; i < count; i++)
  {
    digitalWrite(ERRORLED, HIGH);
    delay(1);
    digitalWrite(ERRORLED, LOW);
    delay(300);
  }
  delay(10000);
}

int SelfTestStart()
{
  //Serial.println("Self test");
  byte a = 10;
  byte b = 170;
  byte c = 100;
  int delayTime = 500;
  int phMax = 1023;
  int phMin = 0;

  // US sensors
  if(GetDistanceInCentimetersLeftSensor() == 3.0)
  {
    return LEFTUSSENSORERROR;
  }
  delay(10);
  if(GetDistanceInCentimetersRightSensor() == 3.0)
  {
    return RIGHTUSSENSORERROR;
  }
  delay(10);
  if(GetDistanceInCentimetersCentralSensor() == 3.0)
  {
    return CENTRALUSSENSORERROR;
  }
  delay(10);
  //Photo sensors  
  if(!(bc.GetPhotoSensorData(1) == phMin && bc.GetPhotoSensorData(2) == phMin && bc.GetPhotoSensorData(3) == phMin) &&
        !(bc.GetPhotoSensorData(1) == phMax && bc.GetPhotoSensorData(2) == phMax && bc.GetPhotoSensorData(3) == phMax ))
      {
        if(bc.GetPhotoSensorData(1) == phMin || bc.GetPhotoSensorData(1) == phMax)
        {
          return PHOTOSENSORSOLAR1ERROR;
        }
         if(bc.GetPhotoSensorData(2) == phMin || bc.GetPhotoSensorData(2) == phMax)
        {
          return PHOTOSENSORSOLAR2ERROR;
        }
        if(bc.GetPhotoSensorData(3) == phMin || bc.GetPhotoSensorData(3) == phMax)
        {
          return PHOTOSENSORSOLAR3ERROR;
        }
      }
  delay(10);    
  //US Servo
  servoUltrasoundSensor.write(a);
  delay(delayTime);
  if(servoUltrasoundSensor.read()!= a)
  {
    return SERVOUSSENSORERROR;
  }
  servoUltrasoundSensor.write(b);
  delay(delayTime);
  if(servoUltrasoundSensor.read()!= b)
  {
    return SERVOUSSENSORERROR;
  }
  servoUltrasoundSensor.write(c);
  delay(delayTime);
  
  //Solar servos
  bc.RunServos(servoSunBatteryHorizontal.read(), MIN_SOLAR_HORIZONTAL_ANGLE, servoSunBatteryHorizontal);
  delay(delayTime);
  if(servoSunBatteryHorizontal.read() - MIN_SOLAR_HORIZONTAL_ANGLE > 3)
  {
    return SERVOSOLARHORIZONTALERROR;
  }
  delay(10);
  
  bc.RunServos(servoSunBatteryHorizontal.read(), MAX_SOLAR_HORIZONTAL_ANGLE, servoSunBatteryHorizontal);
  delay(delayTime);
  if(MAX_SOLAR_HORIZONTAL_ANGLE - servoSunBatteryHorizontal.read() > 3)
  {
    return SERVOSOLARHORIZONTALERROR;
  }
  delay(10);
  
  bc.RunServos(servoSunBatteryVertical.read(), MIN_SOLAR_VERTICAL_ANGLE, servoSunBatteryVertical);
  delay(delayTime);
  if(servoSunBatteryVertical.read() - MIN_SOLAR_VERTICAL_ANGLE > 3)
  {
    return SERVOSOLARVERTICALERROR;
  }
  delay(10);
  
  bc.RunServos(servoSunBatteryVertical.read(), MAX_SOLAR_VERTICAL_ANGLE, servoSunBatteryVertical);
  delay(delayTime);
  if(MAX_SOLAR_VERTICAL_ANGLE - servoSunBatteryVertical.read() > 3)
  {
    return SERVOSOLARVERTICALERROR;
  }
  return OK;
}

void OnSoundInterrupt()            //обработка прерывания на порте D2, звуковой сенсор
{
  //Serial.print(".");
  interrupts();
  if(millis() - interruptorTime > 1000)
  {
    interruptorTime = millis();
    enginesEnabled = !enginesEnabled;
    OnStart();    
    //Serial.println("interrupt");
  }
}

void WakeUpNow()                   //обработка прерывания на порте D3, фотосенсор
{ }

void SleepNow()                 //режим сна
{ 
  sendCommand.SetSleepModeCmd();
  delay(1000);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  attachInterrupt(1, WakeUpNow, LOW);
  globalMode = SLEEP;
  sleep_mode();
  //отслюда после пробуждения
  sleep_disable();
  detachInterrupt(1);
  interrupts();
  WakeUpShields();
}

void WakeUpShields()            // послать сигнал проснуться на другие контроллеры
{
  digitalWrite(OUTPUT_WAKEUP_INTERRUPT_PIN, 1);
  delay(10);
  digitalWrite(OUTPUT_WAKEUP_INTERRUPT_PIN, 0);
}

bool IsParkedForSleep()                                   // парковка
{
  servoUltrasoundSensor.write(100);
  delay(200);
  float distanceForward = GetDistanceInCentimetersCentralSensor();
  while (distanceForward > 35)
  {    
    distanceForward = GetDistanceInCentimetersCentralSensor();
  }
  sendCommand.StopTankCmd();
  return true;
}

void CheckForObstackles()                                // поиск препятствий
{      
  float distanceLeftDown = GetDistanceInCentimetersLeftSensor();
  float distanceRightDown = GetDistanceInCentimetersRightSensor();

  if(distanceLeftDown < 10 || distanceRightDown < 10)
  {
    //Serial.println("! obstacle");
    sendCommand.StopTankCmd();    
    sendCommand.TurnBackCmd();
    delay(100);
    sendCommand.StopTankCmd();
    TurnRightOrLeft(); 
  }
  if(distanceLeftDown < 25)
  {
    sendCommand.TurnRightCmd();
  }
  if(distanceRightDown < 25)
  {
    sendCommand.TurnLeftCmd();
  }
  
  servoUltrasoundSensor.write(100);
  delay(300);
  float distanceForward = GetDistanceInCentimetersCentralSensor();
  
  if (distanceForward < 25)
  {
    //Serial.println("distanceForward"); Serial.println(distanceForward);
    sendCommand.StopTankCmd();
    sendCommand.TurnBackCmd();
  }
  if (distanceForward < 35)
  {
    sendCommand.StopTankCmd();
    TurnRightOrLeft();
  }  
  else
  {    
    sendCommand.MoveForwardCmd();
  }   

}

void TurnRightOrLeft()                                // выбор стороны поворота
{
  if (millis() - dTforUSsensor > 10000)
  {
    dTforUSsensor = millis();
    if (actionsCounter > 6)
    {
      //Serial.println("actionsCounter > 6");
      sendCommand.TurnBackCmd();
      sendCommand.MoveBackCmd();
    }
    actionsCounter = 0;
  }
  servoUltrasoundSensor.write(20);
  delay(300);
  float distanceRight = GetDistanceInCentimetersCentralSensor();
  servoUltrasoundSensor.write(170);
  delay(300);
  float distanceLeft = GetDistanceInCentimetersCentralSensor();
  servoUltrasoundSensor.write(100);

  if(distanceRight < 30 && distanceLeft < 30)
  {
    //Serial.println("distanceRight < 30 && distanceLeft < 30");
    sendCommand.TurnBackCmd();
  }  
  else if (distanceRight > distanceLeft)
  {
    sendCommand.TurnRightCmd();
    actionsCounter ++;
  }
  else 
  {
    sendCommand.TurnLeftCmd();
    actionsCounter ++;
  }  
}

float GetDistanceInCentimetersCentralSensor()                       //получить расстояние с ультрозв. датчика
{
  digitalWrite(ULTRASOUND_CENTRAL_SENSOR_TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASOUND_CENTRAL_SENSOR_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASOUND_CENTRAL_SENSOR_TRIGGER_PIN, LOW);
  float distance = (pulseIn(ULTRASOUND_CENTRAL_SENSOR_ECHO_PIN, HIGH)) / 58.2;    
  //Serial.print("distance = "); Serial.println(distance);
  if (distance > 200)distance = 200;
  if (distance < 3)distance = 3;
  return distance;  
}

float GetDistanceInCentimetersLeftSensor()
{
  digitalWrite(ULTRASOUND_LEFT_SENSOR_TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASOUND_LEFT_SENSOR_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASOUND_LEFT_SENSOR_TRIGGER_PIN, LOW);
  float distance = (pulseIn(ULTRASOUND_LEFT_SENSOR_ECHO_PIN, HIGH)) / 58.2;
   //Serial.print("dL = "); Serial.println(distance);
  if (distance > 200)distance = 200;
  if (distance < 3)distance = 3;
  return distance; 
}

float GetDistanceInCentimetersRightSensor()
{
  digitalWrite(ULTRASOUND_RIGHT_SENSOR_TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASOUND_RIGHT_SENSOR_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASOUND_RIGHT_SENSOR_TRIGGER_PIN, LOW);
  float distance = (pulseIn(ULTRASOUND_RIGHT_SENSOR_ECHO_PIN, HIGH)) / 58.2;
  //Serial.print("dR = "); Serial.println(distance);
  if (distance > 200)distance = 200;
  if (distance < 3)distance = 3;
  return distance;
}

void TurnOnOffLight()                                                                 //влючить/выключить свет
{ 
  if(globalMode != SUNON)
  {
    bc.SetUpSolarBattery(servoSunBatteryVertical, servoSunBatteryHorizontal);
  }
  if (bc.GetPhotoSensorData(1) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT &&
      bc.GetPhotoSensorData(2) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT &&
      bc.GetPhotoSensorData(3) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT )

  {
    sendCommand.TurnOnTheLightCmd();
  }
  else 
  {    
    sendCommand.PutOutTheLightCmd();
  }
}
