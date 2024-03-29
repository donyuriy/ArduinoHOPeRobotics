
// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------- MASTER(SENSORS) ----------------------------------------
//Libraries
#include <Servo.h>
#include <Wire.h>
#include <avr/sleep.h>

//Commands for I2C interface
#define DTM 99                         // 99 - остановить выполнение всех функций на устройстве на 1000 мс
#define EMP 100                        // 100 - НИЧЕГО, НУЛЬ
#define SLEEP 102                      // 102 - отправить контроллеры в сон SLEEP
#define WUP 103                        // 103 - разбудить контроллеры WAKE UP
#define TEST 104                       // 104 - тест всех датчиков и приводов
#define RST 109                        // 19 - комманда RESET
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
#define MGM 140                         // 140 - получить показания магнитометра
#define SUNON 141                       // 141 - режим СОЛНЕЧНОЙ БАТАРЕИ ВКЛЮЧЁН
#define SUNOFF 142                      // 142 - режим СОЛНЕЧНОЙ БАТАРЕИ ВЫКЛЮЧЕН        глобальный 

//Error codes
#define OK 200
#define MEGNETOMETERDATAERROR 255

#define LEFTUSSENSORERROR 401
#define RIGHTUSSENSORERROR 402
#define CENTRALUSSENSORERROR 403
#define SERVOUSSENSORERROR 411
#define SERVOSOLARHORIZONTALERROR 412
#define SERVOSOLARVERTICALERROR 413
#define PHOTOSENSORSOLAR1ERROR 421
#define PHOTOSENSORSOLAR2ERROR 422
#define PHOTOSENSORSOLAR3ERROR 423

//System variables
#define MAGNETOMETR_HMC5883_MESASURING_COMMAND1 0x0B      // Tell the HMC5883 to Continuously Measure
#define MAGNETOMETR_HMC5883_MESASURING_COMMAND2 0x09      // Tell the HMC5883 to Continuously Measure
#define MAGNETOMETR_HMC5883_ADDRESS 0x0D                  //I2C Address for The HMC5883 magnetometer
#define MAGNETOMETER_REGISTER_1 0x01                      // Set the Register 1
#define MAGNETOMETER_REGISTER_2 0x1D                      // Set the Register 2
#define MAGNETOMETER_REGISTER_3 0x00                      // Set the Register 3
#define MASTER_DEVICE_SENSORS 0x64                        // I2C-номер данного устройства
#define SLAVE_DEVICE_CHASIS 0x65                          // I2C-номер устройства шасси
#define SLAVE_DEVICE_102 0x66                             // I2C-номер устройства 102

#define LOWEST_BATTERY_CHARGE 2.51                        // значение соответствует напряжению 2.93 вольта 
#define LOW_BATTERY_CHARGE 2.92                           // значение соответствует напряжению 3.43 вольта  (остаток 10% )
#define HIGH_BATTERY_CHARGE 3.3                           // значение соответствует напряжению 4 вольта
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_SOLAR_BATTERY 200    // значение освещенности для ВКЛЮЧЕНИЯ Солнечной Батареи
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_AWAKE 400            // значение освещенности для ПРОСНУТЬСЯ
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT 600     // значение освещенности для ВКЛЮЧЕНИЯ ОСВЕЩЕНИЯ
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP 800            // значение освещенности для перехода в РЕЖИМ СНА
#define MIN_SOLAR_VERTICAL_ANGLE 70                       // минимальный угол поворота по вертикали ^
#define MAX_SOLAR_VERTICAL_ANGLE 120                      // максимальный угол поворота по вертикали ^
#define MIN_SOLAR_HORIZONTAL_ANGLE 5                      // минимальный угол поворота Солнечной Панели по горизонтали <- |
#define MAX_SOLAR_HORIZONTAL_ANGLE 175                    // максимальный угол поворота Солнечной Панели по горизонтали | ->
#define COMMAND_DELAY_TIME 1000                           // время задержки по комманде DTM ( 99 )

//Used PINs
#define ERRORLED 0                                  // показатель ошибки при самотестировании
#define COLLISION_SENSOR 2                          // сенсор столкновения
#define INTERRUPT_1_DIGITAL_PIN 3                   // порт, обрабатывающий прерывание INTERRUPT_1_PIN
#define INTERRUPT_0_PIN 0                           // порт для обработки прерываний D2 (interrupt #0) - датчик столкновения
#define INTERRUPT_1_PIN 1                           // порт для обработки прерываний D3 (interrupt #1) - цифровой сенсор освещенности
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
#define SOLAR_SENSOR_PIN_3 A1                       // сенсор освещенности 3
#define SOLAR_SENSOR_PIN_2 A2                       // сенсор освещенности 1
#define SOLAR_SENSOR_PIN_1 A3                       // сенсор освещенности 2

//Global variables
unsigned long dTforUSsensor = 0;                   // задержка времени для предотврашения застреваня в узких для поворота местах
unsigned long dTlight = 0;                         // задержка времени . ВКЛ/ВЫКЛ освещение
unsigned long dTvoltage = 0;                       // задержка времени . проверка заряда батареи
unsigned long dTsolar = 0;                         // задержка времени . период перекалибровки положения солнечной панели
unsigned long timeToSleep = 0;                     // оживание до сасыпания
volatile unsigned long interruptorTime = 0;        // для срабатывание функции прерывания на d2 (OnSoundInterrupt)

byte actionsCounter = 0;                           // количество повторяющихся поворотов за последние N секунд
byte motionDirectionMode = EMP;                    // последний установленный режим (см. пометку "глобальный")
byte lightingMode = EMP;                           // дополнительный режим
byte sunBatteryMode = EMP;                         // режим солнечной батареи

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
    void GetMagnetometerValues();
    void RunTest();

  private:
    void SendCommandToChasis(byte command);
    void SendCommandTo102(byte command);
};

class BatteryClass
{
  public:
    BatteryClass()
    {
      photosensorDefference = 2;
    }
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
    byte photosensorDefference;                    // разница показаний фотосенсоров для операций Солнечной батареи
    byte verticalSunBattery_angle;                 // положение вертикального двигателя солнечной батареи
    byte horizontalSunBattery_angle;               // положение горизонтального двигателя солнечной батареи
};

class TestClass
{
  public:
    TestClass()
    {
      testAttemptsLeft = 5;                       // попыток для прохождения самотестирования
      errorLevel = 0;                             // ошибка в процессе тестирования
    }

    ~TestClass();
    void Flasher(byte count);
    void RunSelfTest();
    byte testAttemptsLeft;                          // попыток для прохождения самотестирования
    int errorLevel;                                 // ошибка в процессе тестирования

  private:
    int UsSensorsTestRun();
    int PhotoSensorsTestRun();
    int UsServosTestRun();
    int SolarServosTestRun();
    void ChasisModuleTestRun();
    void HandleError(int error);
};

BatteryClass bc;
Command cmd;
TestClass tests;

Servo servoUltrasoundSensor;
Servo servoSunBatteryVertical;
Servo servoSunBatteryHorizontal;

void setup()
{
  //Serial.begin(9600);
  Wire.begin(MASTER_DEVICE_SENSORS);
  Wire.onReceive(OnReceiveEventHandler);
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
  pinMode(INTERRUPT_1_DIGITAL_PIN, INPUT);
  pinMode(COLLISION_SENSOR, INPUT_PULLUP);
  servoUltrasoundSensor.attach(SERVO_ULTRASOUND_SENSOR_PIN);
  servoSunBatteryVertical.attach(SERVO_SUN_BATTERY_MOTOR_1);
  servoSunBatteryHorizontal.attach(SERVO_SUN_BATTERY_MOTOR_2);
  delay(100);
  OnStart();
  interruptorTime = millis();
  digitalWrite(ERRORLED, LOW);
}

void OnStart()
{
  tests.RunSelfTest();
  cmd.ResetCmd();
  bc.CheckBatteryVoltage();
}

void loop()
{
  if (tests.errorLevel == OK)
  {
    if (sunBatteryMode != SUNON)
    {
      CheckForObstackles();
    }
    else
    {
      cmd.StopTankCmd();
    }

    if (millis() - dTlight > 2000)
    {
      dTlight = millis();
      TurnOnOffLight();
    }

    if (millis() - dTvoltage > 10000)
    {
      dTvoltage = millis();
      bc.CheckBatteryVoltage();

      if (sunBatteryMode == SUNON)
      {
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
    digitalWrite(ERRORLED, LOW);
  }
  else if (tests.testAttemptsLeft > 0)
  {
    Serial.print("tests.testAttemptsLeft: "); Serial.println(tests.testAttemptsLeft);
    tests.testAttemptsLeft --;
    tests.RunSelfTest();
  }
  else
  {
    Serial.print("tests.testAttemptsLeft: "); Serial.println(tests.testAttemptsLeft);
    Serial.print("Error level: "); Serial.println(tests.errorLevel);
    tests.Flasher(3);
  }
}

void OnReceiveEventHandler(int howMany)
{
  if (Wire.available() > 0)
  {
    byte in_data = Wire.read();
    interrupts();
    ChooseAction(in_data);
  }
}

void ChooseAction(byte in_data)
{
  //Serial.print("Master in_data: "); Serial.println(in_data);
  switch (in_data)
  {
    case DTM:
      DelayController();
      break;
    case MEGNETOMETERDATAERROR:
      tests.errorLevel = MEGNETOMETERDATAERROR;
      break;
    default:
      break;
  }
}

void WakeUpNow()                   //обработка прерывания на порте D3, фотосенсор
{ }

void SleepNow()                 //режим сна
{
  cmd.SetSleepModeCmd();
  delay(1000);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  attachInterrupt(INTERRUPT_1_PIN, WakeUpNow, LOW);   // цифровой датчик освещения (сигнал проснуться)
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
  cmd.StopTankCmd();
  return true;
}

void CheckForObstackles()                                // поиск препятствий
{
  CheckForCollision();
  float distanceLeftDown = GetDistanceInCentimetersLeftSensor();
  float distanceRightDown = GetDistanceInCentimetersRightSensor();

  if (distanceLeftDown < 10 || distanceRightDown < 10)
  {
    //Serial.println("! obstacle");
    cmd.StopTankCmd();
    cmd.TurnBackCmd();
    delay(100);
    cmd.StopTankCmd();
    TurnRightOrLeft();
  }
  if (distanceLeftDown < 25)
  {
    cmd.TurnRightCmd();
  }
  if (distanceRightDown < 25)
  {
    cmd.TurnLeftCmd();
  }

  servoUltrasoundSensor.write(100);
  delay(300);
  float distanceForward = GetDistanceInCentimetersCentralSensor();

  if (distanceForward < 25)
  {
    //Serial.println("distanceForward"); Serial.println(distanceForward);
    cmd.StopTankCmd();
    cmd.TurnBackCmd();
  }
  if (distanceForward < 35)
  {
    cmd.StopTankCmd();
    TurnRightOrLeft();
  }
  else
  {
    cmd.MoveForwardCmd();
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
      cmd.TurnBackCmd();
      cmd.MoveBackCmd();
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

  if (distanceRight < 30 && distanceLeft < 30)
  {
    //Serial.println("distanceRight < 30 && distanceLeft < 30");
    cmd.TurnBackCmd();
  }
  else if (distanceRight > distanceLeft)
  {
    cmd.TurnRightCmd();
    actionsCounter ++;
  }
  else
  {
    cmd.TurnLeftCmd();
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

void CheckForCollision()
{
  bool isCollision = !digitalRead(COLLISION_SENSOR);
  if (isCollision)
  {
    cmd.StopTankCmd();
    cmd.MoveBackCmd();
    TurnRightOrLeft();
  }
}

void TurnOnOffLight()                                                                 //влючить/выключить свет
{
  if (sunBatteryMode != SUNON)
  {
    bc.SetUpSolarBattery(servoSunBatteryVertical, servoSunBatteryHorizontal);
  }
  if (bc.GetPhotoSensorData(1) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT &&
      bc.GetPhotoSensorData(2) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT &&
      bc.GetPhotoSensorData(3) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT)
  {
    cmd.TurnOnTheLightCmd();
  }
  else
  {
    cmd.PutOutTheLightCmd();
  }
}

void DelayController()
{
  delay(COMMAND_DELAY_TIME);
}
