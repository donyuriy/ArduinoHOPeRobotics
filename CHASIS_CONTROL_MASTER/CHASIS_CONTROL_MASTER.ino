// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------- MASTER ----------------------------------------

#include <Servo.h>
#include <Wire.h>
#include <avr/sleep.h>

#define EMP 100                         // 10 - НИЧЕГО, НУЛЬ
#define SLEEP 102                       // 12 - отправить контроллеры в сон SLEEP
#define WUP 103                         // 13 - разбудить контроллеры WAKE UP
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

#define SLAVE_DEVICE_CHASIS 0x65
#define SLAVE_DEVICE_CAMERA 0x66
#define LOWEST_BATTERY_CHARGE 2.51                        // значение соответствует напряжению 2.93 вольта 
#define LOW_BATTERY_CHARGE 2.92                           // значение соответствует напряжению 3.43 вольта  (остаток 10% )
#define HIGH_BATTERY_CHARGE 3.3                           // значение соответствует напряжению 4 вольта
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_SOLAR_BATTERY 200    // значение освещенности для ВКЛЮЧЕНИЯ Солнечной Батареи
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_AWAKE 400            // значение освещенности для ПРОСНУТЬСЯ
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT 600     // значение освещенности для ВКЛЮЧЕНИЯ ОСВЕЩЕНИЯ
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP 800            // значение освещенности для перехода в РЕЖИМ СНА
#define MIN_SOLAR_VERTICAL_ANGLE 110                      // минимальный угол поворота по вертикали
#define MAX_SOLAR_VERTICAL_ANGLE 160                      // максимальный угол поворота по вертикали
#define MIN_SOLAR_HORIZONTAL_ANGLE 5                      // минимальный угол поворота Солнечной Панели по горизонтали <- |
#define MAX_SOLAR_HORIZONTAL_ANGLE 170                    // максимальный угол поворота Солнечной Панели по горизонтали | ->

#define INTERRUPT_0_PIN 2                           // порт для обработки прерываний D2 (interrupt #0)
#define INTERRUPT_1_PIN 3                           // порт для обработки прерываний D3 (interrupt #1)
#define IR_OBSATACLE_SENSOR_1_PIN 4                 // ИК-сенсор препятствий 1
#define IR_OBSATACLE_SENSOR_2_PIN 5                 // ИК-сенсор препятствий 2
#define IR_OBSATACLE_SENSOR_3_PIN 6                 // ИК-сенсор препятствий 3
#define IR_OBSATACLE_SENSOR_4_PIN 7                 // ИК-сенсор препятствий 4
#define SERVO_SUN_BATTERY_MOTOR_1 8                 // сервопривод управления Солнечной панелью (горизонт)
#define SERVO_SUN_BATTERY_MOTOR_2 9                 // сервопривод управления Солнечной панелью (вертикаль)
#define SERVO_ULTRASOUND_SENSOR_PIN 10              // сервопривод управления УЗ-сенсором расстояния
#define ULTRASOUND_SENSOR_TRIGGER_PIN 11            // УЗ-сенсор расстояния передатчик
#define ULTRASOUND_SENSOR_ECHO_PIN 12               // УЗ-сенсор расстояния приёмник
#define OUTPUT_WAKEUP_INTERRUPT_PIN 13              // для отправки цифрового сигнала для прерывания пробуждения вспомогательных шилдов
#define VOLTMETER_SENSOR_PIN A0                     // вольтметр батареи
#define SOLAR_SENSOR_PIN_2 A2                       // сенсор освещенности 1
#define SOLAR_SENSOR_PIN_1 A3                       // сенсор освещенности 2
#define SOLAR_SENSOR_PIN_3 A1                       // сенсор освещенности 3

unsigned long dTforUSsensor = 0;                   // задержка времени для предотврашения застреваня в узких для поворота местах
unsigned long dTlight = 0;                         // задержка времени -> ВКЛ/ВЫКЛ освещение
unsigned long dTvoltage = 0;                       // задержка времени -> проверка заряда батареи
unsigned long dTsolar = 0;                         // задержка времени -> период перекалибровки положения солнечной панели
byte actionsCounter = 0;                           // количество повторяющихся поворотов за последние N секунд
byte globalMode = 0;                               // последний установленный режим (см. пометку "глобальный")
byte extraMode = 0;                                // дополнительный режим
byte verticalSunBattery_angle;                     // положение вертикального двигателя солнечной батареи
byte horizontalSunBattery_angle;                   // положение горизонтального двигателя солнечной батареи
byte angleDifference = 2;                          // разница показаний сервоприводов для операций Солнечной батареи
byte photosensorDefference = 2;                    // разница показаний фотосенсоров для операций Солнечной батареи

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
  
  private:  
  void SearchVerticalSolar();
  void SearchHorizontalSolar();  
  void SolarSearchingInMotion();
  void RunServos(byte ServoStartAngle, byte ServoFinishAngle, Servo servo);
};

BatteryClass *bc;
Command *sendCommand;
Servo servoUltrasoundSensor;
Servo servoSunBatteryVertical;
Servo servoSunBatteryHorizontal;

void setup()
{
  //Serial.begin(9600);
  Wire.begin(); 
  pinMode(ULTRASOUND_SENSOR_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASOUND_SENSOR_ECHO_PIN, INPUT);
  pinMode(OUTPUT_WAKEUP_INTERRUPT_PIN, OUTPUT);
  pinMode(IR_OBSATACLE_SENSOR_1_PIN, INPUT);
  pinMode(IR_OBSATACLE_SENSOR_2_PIN, INPUT);
  pinMode(IR_OBSATACLE_SENSOR_3_PIN, INPUT);
  pinMode(IR_OBSATACLE_SENSOR_4_PIN, INPUT);
  pinMode(SOLAR_SENSOR_PIN_1, INPUT);
  pinMode(SOLAR_SENSOR_PIN_2, INPUT);
  pinMode(SOLAR_SENSOR_PIN_3, INPUT);
  pinMode(VOLTMETER_SENSOR_PIN, INPUT);
  pinMode(INTERRUPT_1_PIN, INPUT);
  servoUltrasoundSensor.attach(SERVO_ULTRASOUND_SENSOR_PIN);
  servoSunBatteryVertical.attach(SERVO_SUN_BATTERY_MOTOR_1);
  servoSunBatteryHorizontal.attach(SERVO_SUN_BATTERY_MOTOR_2);  
  attachInterrupt(0, SoundProcessing, CHANGE);  
}

void loop()
{ 
  if(globalMode != SUNON)
  {
    //SpeedCorrection();  
    CheckForObstackles();
  }

  if (millis() - dTlight > 1000)
  {
    dTlight = millis();
    TurnOnOffLight();     
  }

  if (millis() - dTvoltage > 10000)
  {
    dTvoltage = millis();
    bc->CheckBatteryVoltage();
    
    if(globalMode == SUNON)
    {
      dTsolar = millis();
      bc->ActionSolarBatteryOn();
    }
    
    if (bc->GetPhotoSensorData(1) > MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP &&
        bc->GetPhotoSensorData(2) > MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP &&
        bc->GetPhotoSensorData(3) > MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP)
    {
      EnableSleepingMode();
    }
  }
}

void SoundProcessing()                                 //обработка прерывания на порте D2, звуковой сенсор
{
  interrupts();
}

void WakeUpNow()                                       //обработка прерывания на порте D3, фотосенсор
{ }

void EnableSleepingMode()                               //режим сна через 60 секунд
{
  unsigned long timeDelay = millis();  
  while (millis() - timeDelay < 20000)
  {
    //SpeedCorrection();  
    CheckForObstackles();
    if (millis() - dTlight > 1000)
    {
      dTlight = millis();
      TurnOnOffLight();     
    }
  }
  if (IsParkedForSleep())
  {
    if (bc->GetPhotoSensorData(1) < MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP ||
        bc->GetPhotoSensorData(2) < MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP ||
        bc->GetPhotoSensorData(3) < MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP)
    {
      return;
    }
    //SleepNow();
  }
}

void SleepNow()                                         //режим сна
{
  //Serial.println("SleepNow");
  sendCommand->SetSleepModeCmd();
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

void WakeUpShields()                                      // послать сигнал проснуться на другие контроллеры
{
  digitalWrite(OUTPUT_WAKEUP_INTERRUPT_PIN, 1);
  delayMicroseconds(10000);
  digitalWrite(OUTPUT_WAKEUP_INTERRUPT_PIN, 0);
}

bool IsParkedForSleep()                                   // парковка
{
  servoUltrasoundSensor.write(100);
  delay(100);
  float distanceForward = GetDistanceInCentimeters();
  while (distanceForward > 35)
  {
    bool irSensor1_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_1_PIN);
    bool irSensor2_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_2_PIN);
    bool irSensor3_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_3_PIN);
    bool irSensor4_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_4_PIN);

    if (irSensor1_ObstacleFound || irSensor3_ObstacleFound)
    {
      sendCommand->TurnRightCmd();
    }
    else if (irSensor2_ObstacleFound || irSensor4_ObstacleFound)
    {
      sendCommand->TurnLeftCmd();
    }
    distanceForward = GetDistanceInCentimeters();
  }
  sendCommand->StopTankCmd();
  return true;
}

void SpeedCorrection()
{
  unsigned long startTime = millis();
  servoUltrasoundSensor.write(100);
  delay(200);
  float distanceForward1 = GetDistanceInCentimeters(); 
  delay(50);
  float distanceForward2 = GetDistanceInCentimeters();
  unsigned long deltaTime = (millis() - startTime);
  float dDistance = abs(distanceForward1 - distanceForward2); 
  
  if(dDistance*1000/deltaTime > 25)
  {    
    sendCommand->SlowDownCmd();
  }
  if(dDistance*1000/deltaTime > 20)
  {
    sendCommand->SlowDownCmd();
  }
  else if(dDistance*1000/deltaTime < 16)
  {
    sendCommand->SpeedUpCmd();
  }
}

void CheckForObstackles()                                // поиск препятствий
{
  //Serial.println("CheckForObstackles");
  bool irSensor1_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_1_PIN);
  bool irSensor2_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_2_PIN);
  bool irSensor3_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_3_PIN);
  bool irSensor4_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_4_PIN);
  if ((irSensor1_ObstacleFound && irSensor2_ObstacleFound)
      || (irSensor3_ObstacleFound && irSensor4_ObstacleFound))
  {
    sendCommand->StopTankCmd();
    TurnRightOrLeft();
  }
  else if (irSensor1_ObstacleFound || irSensor3_ObstacleFound)
  {
    sendCommand->TurnRightCmd();
  }
  else if (irSensor2_ObstacleFound || irSensor4_ObstacleFound)
  {
    sendCommand->TurnLeftCmd();
  }

  servoUltrasoundSensor.write(100);
  delay(200);
  float distanceForward = GetDistanceInCentimeters();
  
  if (distanceForward < 35)
  {
    sendCommand->StopTankCmd();
    TurnRightOrLeft();
  }
  if (distanceForward < 20)
  {
    sendCommand->StopTankCmd();
    sendCommand->MoveBackCmd();
  }
  else
  {
    sendCommand->MoveForwardCmd();
  }
}

void TurnRightOrLeft()                                // выбор стороны поворота
{
  if (millis() - dTforUSsensor > 7000)
  {
    dTforUSsensor = millis();
    if (actionsCounter > 5)
    {
      sendCommand->TurnBackCmd();
      sendCommand->MoveBackCmd();
    }
    actionsCounter = 0;
  }
  servoUltrasoundSensor.write(20);
  delay(250);
  float distanceRight = GetDistanceInCentimeters();
  servoUltrasoundSensor.write(180);
  delay(250);
  float distanceLeft = GetDistanceInCentimeters();
  servoUltrasoundSensor.write(100);

  if (distanceRight < 30 && distanceLeft < 30)
  {
    sendCommand->MoveBackCmd();
  }
  if (distanceRight > distanceLeft)
  {
    sendCommand->TurnRightCmd();
    actionsCounter ++;
  }
  else
  {
    sendCommand->TurnLeftCmd();
    actionsCounter ++;
  }  
}

float GetDistanceInCentimeters()                       //получить расстояние с ультрозв. датчика
{
  digitalWrite(ULTRASOUND_SENSOR_TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASOUND_SENSOR_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASOUND_SENSOR_TRIGGER_PIN, LOW);
  int distance = (pulseIn(ULTRASOUND_SENSOR_ECHO_PIN, HIGH)) / 58.2;
  if (distance > 200)distance = 200;
  if (distance < 5)distance = 5;
  return distance;
}

void TurnOnOffLight()                                                                 //влючить/выключить свет
{ 
  //Serial.println("TurnOnOffLight");
  if(globalMode != SUNON)
  {
    bc->SetUpSolarBattery(servoSunBatteryVertical, servoSunBatteryHorizontal);
  }
  if (bc->GetPhotoSensorData(1) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT &&
      bc->GetPhotoSensorData(2) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT &&
      bc->GetPhotoSensorData(3) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT )
  {
    sendCommand->TurnOnTheLightCmd();
  }
  else 
  {    
    sendCommand->PutOutTheLightCmd();
  }
}
