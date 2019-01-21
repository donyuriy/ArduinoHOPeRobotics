// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------- MASTER ----------------------------------------
#include <Servo.h>
#include <Wire.h>
#include <avr/sleep.h>
 
#define EMP 0         // 0 - отсутствие комманды,
#define SLEEP 2       // 2 - отправить контроллеры в СОН
#define WUP 3         // 3 - РАЗБУДИТЬ контроллеры
#define RST 9         // 9 - комманда ОБНУЛИТЬ
#define STP 10        // 10 - СТОП
#define FWD 11        // 11 - ВПЕРЕД
#define BWD 12        // 12 - НАЗАД
#define SPU 15        // 15 - УСКОРИТЬ
#define SDN 16        // 16 - ЗАМЕДЛИТЬ
#define LFT 21        // 21 - поворот НАЛЕВО
#define RGT 22        // 22 - поворот НАПРАВО
#define TBK 23        // 23 - РАЗВОРОТ на 180
#define TLT 31        // 31 - ВКЛЮЧИТЬ СВЕТ
#define PLT 32        // 32 - ВЫКЛЮЧИТЬ СВЕТ
#define SHB 33        // 33 - сделать подсветку ЯРЧЕ
#define SHD 34        // 34 - сделать подсветку ТУСКЛЕЕ
#define SUNON 41      // 41 - режим СОЛНЕЧНОЙ БАТАРЕИ ВКЛЮЧЁН
#define SUNOFF 42     // 42 - режим СОЛНЕЧНОЙ БАТАРЕИ ВЫКЛЮЧЕН    

#define SLAVE_DEVICE_CHASIS 101
#define SLAVE_DEVICE_SENSORS 102
#define LOW_BATTERY_CHARGE 2.2                            // значение соответствует напряжению 3.5 вольта  
#define HIGH_BATTERY_CHARGE 3.2                           // значение соответствует напряжению 4 вольта
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_SOLAR_BATTERY 100    // значение освещенности для включения Солнечной батареи
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT 600     // значение освещенности для включения освещения (чем хуже освещенность, тем выше сигнал)
#define MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP 900            //значение освещенности для перехода в режим сна
#define MIN_SOLAR_VERTICAL_ANGLE 100
#define MAX_SOLAR_VERTICAL_ANGLE 160
#define MIN_SOLAR_HORIZONTAL_ANGLE 5
#define MAX_SOLAR_HORIZONTAL_ANGLE 175


#define INTERRUPT_0_PIN 2                 // порт для обработки прерываний D2 (interrupt #0)
#define INTERRUPT_1_PIN 3                 // порт для обработки прерываний D3 (interrupt #1)
#define IR_OBSATACLE_SENSOR_1_PIN 4       // ИК-сенсор препятствий 1
#define IR_OBSATACLE_SENSOR_2_PIN 5       // ИК-сенсор препятствий 2
#define IR_OBSATACLE_SENSOR_3_PIN 6       // ИК-сенсор препятствий 3
#define IR_OBSATACLE_SENSOR_4_PIN 7       // ИК-сенсор препятствий 4
#define SERVO_SUN_BATTERY_MOTOR_1 8       // сервопривод управления Солнечной панелью (горизонт)
#define SERVO_SUN_BATTERY_MOTOR_2 9       // сервопривод управления Солнечной панелью (вертикаль)
#define SERVO_ULTRASOUND_SENSOR_PIN 10    // сервопривод управления УЗ-сенсором расстояния
#define ULTRASOUND_SENSOR_TRIGGER_PIN 11  // УЗ-сенсор расстояния передатчик
#define ULTRASOUND_SENSOR_ECHO_PIN 12     // УЗ-сенсор расстояния приёмник
#define OUTPUT_WAKEUP_INTERRUPT_PIN 13    // 
#define VOLTMETER_SENSOR_PIN A0           // вольтметр батареи
#define SOLAR_SENSOR_PIN_2 A2             // сенсор освещенности 1
#define SOLAR_SENSOR_PIN_1 A3             // сенсор освещенности 2

Servo servoUltrasoundSensor;
Servo servoSunBatteryVertical;
Servo servoSunBatteryHirizontal;

unsigned long dTforUSsensor = 0;              // задержка времени для предотврашения застреваня в узких для поворота местах
unsigned long dTloop = 0;                     // задержка времени -> основной для цикла loop()
unsigned long dTvolatge = 0;                  // задержка времени -> проверка заряда батареи
byte actionsCounter = 0;                      // количество повторяющихся поворотов за последние N секунд
byte mode = 0;                                // последний установленный режим
byte verticalSunBattery_angle = 0;            // положение вертикального двигателя солнечной батареи
byte horizontalSunBattery_angle = 0;          // положение горизонтального двигателя солнечной батареи
int photosensorsData[28];                     // суммарная освещенность всех фото-сенсоров

void setup()
{
  Wire.begin();
  Wire.onRequest(OnRequestEventHandler);
  pinMode(ULTRASOUND_SENSOR_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASOUND_SENSOR_ECHO_PIN, INPUT);
  pinMode(OUTPUT_WAKEUP_INTERRUPT_PIN, OUTPUT);
  pinMode(IR_OBSATACLE_SENSOR_1_PIN, INPUT);
  pinMode(IR_OBSATACLE_SENSOR_2_PIN, INPUT);
  pinMode(IR_OBSATACLE_SENSOR_3_PIN, INPUT);
  pinMode(IR_OBSATACLE_SENSOR_4_PIN, INPUT);
  pinMode(SOLAR_SENSOR_PIN_1,INPUT);         
  pinMode(SOLAR_SENSOR_PIN_2,INPUT);  
  pinMode(VOLTMETER_SENSOR_PIN, INPUT);
  servoUltrasoundSensor.attach(SERVO_ULTRASOUND_SENSOR_PIN);
  attachInterrupt(0, SoundProcessing, CHANGE);  
  SendCommandToChasis(RST); 
  if(GetPhotoSensorData(1) > MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP && GetPhotoSensorData(2) > MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP)
  {
    SleepNow();
  }
}

void loop()
{   
  if(mode != SUNON)
  {
     CheckForObstackles();      
  }   

  if(millis() - dTloop > 1000)
  {    
     dTloop = millis();          
     TurnOnOffLight();    
  }   

  if(millis() - dTvolatge > 60000)
  {
    dTvolatge = millis();
    CheckBatteryVoltage();
    if(GetPhotoSensorData(1) > MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP && GetPhotoSensorData(2) > MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP)
    {
      EnableSleepingMode();
    }
  } 
}

void OnRequestEventHandler()
{
  interrupts();
  byte response = Wire.read();
  switch(response) 
  {
    default:
      break; 
  }  
  response = 0;
}

void SoundProcessing()                                 //обработка прерывания на порте D2, звуковой сенсор
{                                                    
  interrupts();
}

void WakeUpNow()                                       //обработка прерывания на порте D3, фотосенсор  
{    
   
}

void SendCommandToChasis(byte command)                 //отправка команд на шасси
{
  Wire.beginTransmission(SLAVE_DEVICE_CHASIS);
  Wire.write(command);
  Wire.endTransmission();
  Wire.requestFrom(SLAVE_DEVICE_CHASIS, 1);
}

void EnableSleepingMode()                               //режим сна через 5 минут (300 секунд)                       
{
  unsigned long timeDelay = millis() + 300000;
  while(timeDelay - millis() > 0)
  {
    CheckForObstackles();
    //
    //
    //  СЮДА ВОТКНУТЬ КОД ПО ПАРКОВКЕ ПЕДАЛЬНОГО КОНЯ !
    //
    //  СЮДА ВОТКНУТЬ КОД ПО ПАРКОВКЕ ПЕДАЛЬНОГО КОНЯ !
    //
    // СЮДА ВОТКНУТЬ КОД ПО ПАРКОВКЕ ПЕДАЛЬНОГО КОНЯ !
    //
    // СЮДА ВОТКНУТЬ КОД ПО ПАРКОВКЕ ПЕДАЛЬНОГО КОНЯ !
    //
  }
  
  if(GetPhotoSensorData(1) < MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP || GetPhotoSensorData(1) < MINIMAL_BRIGHTNESS_LEVEL_FOR_SLEEP)
  {
    return;
  }
  SleepNow();
}

void SleepNow()                                         //режим сна
{
  StopTankCmd();
  mode = SLEEP;
  SetSleepModeCmd();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   
  sleep_enable();         
  attachInterrupt(1,WakeUpNow, LOW);  
  sleep_mode();            
                //отслюда после пробуждения 
  sleep_disable();  
  detachInterrupt(1);  
  interrupts();
  digitalWrite(OUTPUT_WAKEUP_INTERRUPT_PIN, 1);
  delayMicroseconds(50000);      
  digitalWrite(OUTPUT_WAKEUP_INTERRUPT_PIN, 0); 
  
}

void CheckForObstackles()                                // поиск препятствий
{  
  bool irSensor1_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_1_PIN);
  bool irSensor2_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_2_PIN);
  bool irSensor3_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_3_PIN);
  bool irSensor4_ObstacleFound = !digitalRead(IR_OBSATACLE_SENSOR_4_PIN);
    
  if((irSensor1_ObstacleFound && irSensor2_ObstacleFound) 
      ||(irSensor3_ObstacleFound && irSensor4_ObstacleFound))
  {
    StopTankCmd();
    TurnRightOrLeft();
  }
  else if(irSensor1_ObstacleFound || irSensor3_ObstacleFound)
  {
     TurnRightCmd();
  }
  else if(irSensor2_ObstacleFound || irSensor4_ObstacleFound)
  { 
    TurnLeftCmd();
  }
    
  servoUltrasoundSensor.write(100);                        
  delay(200);
  float distanceForward = GetDistanceInCentimeters();
  if(distanceForward < 7)
  {
    StopTankCmd();
    MoveBackCmd();
  }
  else if(distanceForward < 30)
  {
    StopTankCmd();
    TurnRightOrLeft();
  }
  else if(distanceForward < 60)
  {
    TurnRightOrLeft();
  }    
  else
  {
    MoveForwardCmd();
  }
}

void TurnRightOrLeft()                                // выбор стороны поворота
{      
  if(millis() - dTforUSsensor > 10000)
  {
    dTforUSsensor = millis();
    if(actionsCounter > 7)
    {
      TurnBackCmd();
    }
    actionsCounter = 0;
  }  
  servoUltrasoundSensor.write(20);                      
  delay(350);
  float distanceRight = GetDistanceInCentimeters();
  servoUltrasoundSensor.write(180);                     
  delay(350);
  float distanceLeft = GetDistanceInCentimeters();
  servoUltrasoundSensor.write(100);
    
    if(distanceRight < 30 && distanceLeft < 30)
    {
      TurnBackCmd();
    }
    if(distanceRight > distanceLeft)
    {
      TurnRightCmd();
      actionsCounter ++;
    }
    else if(distanceRight < distanceLeft)
    {
      TurnLeftCmd();
      actionsCounter ++;
    }
    else
    {      
      TurnBackCmd();
    }
}


float GetDistanceInCentimeters()                       //получить расстояние с ультрозв. датчика
{   
  digitalWrite(ULTRASOUND_SENSOR_TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASOUND_SENSOR_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASOUND_SENSOR_TRIGGER_PIN, LOW);
  int distance = (pulseIn(ULTRASOUND_SENSOR_ECHO_PIN, HIGH))/58.2;  
  if(distance > 200)distance = 200;
  if(distance < 5)distance = 5;
  return distance;
}

float GetBattaryVoltage()                                     //получить усредненное значение напряжения на батарее
{
  float data = 0;
  byte avarage = 25;
  for(byte i = 0; i < avarage; i ++)
  {
    data += analogRead(VOLTMETER_SENSOR_PIN)*5.0/1024.0;
  }
  return (float)(data/avarage); 
}

bool IsBatteryPowerNormal()                                     // проверка достаточности заряда батареи
{
  if(GetBattaryVoltage() > HIGH_BATTERY_CHARGE) return true;
  else return false;
}

void CheckBatteryVoltage()                                      //проверить заряд БАТАРЕИ
{    
  if(GetBattaryVoltage() < LOW_BATTERY_CHARGE)
  {
    ActionSolarBatteryOn();
  }  
  else if(GetBattaryVoltage() > HIGH_BATTERY_CHARGE)
  {
    ActionSolarBatteryOff();
  }
}

void ActionSolarBatteryOn()                                     //включить Солнечную батарею
{  
  if(GetPhotoSensorData(1) < MINIMAL_BRIGHTNESS_LEVEL_FOR_SOLAR_BATTERY &&
      GetPhotoSensorData(2) < MINIMAL_BRIGHTNESS_LEVEL_FOR_SOLAR_BATTERY &&
      !IsBatteryPowerNormal())
  {
    mode = SUNON;
    StopTankCmd();
    servoSunBatteryVertical.attach(SERVO_SUN_BATTERY_MOTOR_1);
    servoSunBatteryHirizontal.attach(SERVO_SUN_BATTERY_MOTOR_2);
    SearchVerticalSolar();      
    SearchHorizontalSolar(); 
    servoSunBatteryVertical.detach(); 
    servoSunBatteryHirizontal.detach();
  }
  else
  {
    ActionSolarBatteryOff();    
  }
}

void SearchVerticalSolar()
{
  int photoSensor1 = GetPhotoSensorData(1);
  int photoSensor2 = GetPhotoSensorData(2);
  byte verticalServoAngle = 175;
  while(verticalServoAngle != servoSunBatteryVertical.read())
  {
    verticalServoAngle = servoSunBatteryVertical.read();
    if(photoSensor1 - photoSensor2 < -2)
    {
      verticalSunBattery_angle++;
      if(verticalSunBattery_angle < MIN_SOLAR_VERTICAL_ANGLE)verticalSunBattery_angle = MIN_SOLAR_VERTICAL_ANGLE;
      else if(verticalSunBattery_angle > MAX_SOLAR_VERTICAL_ANGLE)verticalSunBattery_angle = MAX_SOLAR_VERTICAL_ANGLE;        
      RunServos(servoSunBatteryVertical.read(),verticalSunBattery_angle, servoSunBatteryVertical);
    }
    else if(photoSensor1 - photoSensor2 > 2)
    {
      verticalSunBattery_angle--;
      if(verticalSunBattery_angle < MIN_SOLAR_VERTICAL_ANGLE)verticalSunBattery_angle = MIN_SOLAR_VERTICAL_ANGLE;
      else if(verticalSunBattery_angle > MAX_SOLAR_VERTICAL_ANGLE)verticalSunBattery_angle = MAX_SOLAR_VERTICAL_ANGLE;        
      RunServos(servoSunBatteryVertical.read(),verticalSunBattery_angle, servoSunBatteryVertical);
    }
  }  
}

void SearchHorizontalSolar()
{
  RunServos(servoSunBatteryHirizontal.read(), MIN_SOLAR_HORIZONTAL_ANGLE, servoSunBatteryHirizontal);
  byte horizontalDifferentialAngleOfRotation = (MAX_SOLAR_HORIZONTAL_ANGLE - MIN_SOLAR_HORIZONTAL_ANGLE)/(sizeof(photosensorsData)/sizeof(photosensorsData[0]));
  photosensorsData[0] = GetPhotoSensorData(1) + GetPhotoSensorData(2);      
  for(byte index = 1; index < (sizeof(photosensorsData)/sizeof(photosensorsData[0])); index ++)
  {
    RunServos(servoSunBatteryHirizontal.read(), MIN_SOLAR_HORIZONTAL_ANGLE + (horizontalDifferentialAngleOfRotation * index), servoSunBatteryHirizontal);
    photosensorsData[index] = GetPhotoSensorData(1) + GetPhotoSensorData(2);
  }
      
  byte maxValueIndex = 0;
  int maxValue = 2000;
  for(byte index = 0; index < (sizeof(photosensorsData)/sizeof(photosensorsData[0])); index ++)
  {
    if(photosensorsData[index] < maxValue)
    {
      maxValue = photosensorsData[index];
      maxValueIndex = index;
    }
  }  
  RunServos(servoSunBatteryHirizontal.read(), (MIN_SOLAR_HORIZONTAL_ANGLE + (horizontalDifferentialAngleOfRotation*(maxValueIndex +1))), servoSunBatteryHirizontal);
}

void ActionSolarBatteryOff()                              //отключить Солнечную батарею
{
  byte X = MIN_SOLAR_VERTICAL_ANGLE + 20;             
  byte Y = MAX_SOLAR_HORIZONTAL_ANGLE - 20;                           
  servoSunBatteryVertical.attach(SERVO_SUN_BATTERY_MOTOR_1);
  servoSunBatteryHirizontal.attach(SERVO_SUN_BATTERY_MOTOR_2);  
  if(servoSunBatteryVertical.read() -  X > 5 || servoSunBatteryVertical.read() -  X < -5)
  {
    RunServos(servoSunBatteryVertical.read(),X, servoSunBatteryVertical);
  }
  if(servoSunBatteryHirizontal.read() -  Y > 5 || servoSunBatteryHirizontal.read() -  Y < -5)
  {
    RunServos(servoSunBatteryHirizontal.read(),Y, servoSunBatteryHirizontal);
  } 
  servoSunBatteryVertical.detach();
  servoSunBatteryHirizontal.detach();  
  mode = EMP;
}

void RunServos(byte Servo1start, byte Servo1finish, Servo servo)  // замедленный поворот сервоприводов
{
  long delayInterval = 25000;
  if(Servo1start < Servo1finish)
  {
    for(byte i = Servo1start; i < Servo1finish; i++)
    {
      servo.write(i);
      delayMicroseconds(delayInterval);
    }
  }
  else
  {
    for(byte i = Servo1start; i > Servo1finish; i--)
    {
      servo.write(i);
      delayMicroseconds(delayInterval);
    }
  }
}

float GetPhotoSensorData(byte sensorID)             // получить показания с аналогового фотосенсора № 1 или 2
{
  int data = 0;
  byte avarage = 25;
  switch(sensorID)
  {
    case 1:
      for(byte i = 0; i < avarage; i++)
      {
        data += analogRead(SOLAR_SENSOR_PIN_1);
      }
      break;
    case 2:
      for(byte i = 0; i < avarage; i++)
      {
        data += analogRead(SOLAR_SENSOR_PIN_2);
      }
      break;
    default:
      break;
  }
  return (float)(data/avarage);
}

void TurnOnOffLight()                                     //влючить/выключить свет
{
  if( GetPhotoSensorData(1) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT &&
  GetPhotoSensorData(2) > MINIMAL_BRIGHTNESS_LEVEL_FOR_TURNON_LIGHT && mode != SUNON)
  {
    TurnOnTheLightCmd();
  }
  else
  {
    PutOutTheLightCmd();
  }
}

void MoveCmd()                            // продолжить движение
{
  if(mode == FWD || mode == BWD)
  {
    SendCommandToChasis(mode);
  }
}

void MoveForwardCmd()                     //двигаться вперёд
{
  mode = FWD;
  SendCommandToChasis(FWD);
}

void MoveBackCmd()                        //двигаться назад
{
  mode = BWD;
  SendCommandToChasis(BWD);
}

void TurnRightCmd()                       //повернуть вправо
{
  mode = RGT;
  SendCommandToChasis(RGT);
}

void TurnLeftCmd()                        //повернуть влево
{
  mode = LFT;
  SendCommandToChasis(LFT);
}

void TurnBackCmd()                        // резвернуться на 180град.
{
  mode = TBK;
  SendCommandToChasis(TBK);
}

void StopTankCmd()                        // остановиться
{  
  mode = STP;
  SendCommandToChasis(STP);
}

void SpeedUpCmd()                         // ускориться
{
  mode = SPU;
  SendCommandToChasis(SPU);
}

void SlowDownCmd()                        // замедлиться
{
  mode = SDN;
  SendCommandToChasis(SDN);
}

void TurnOnTheLightCmd()                  //включить освещение
{
  mode = TLT;
  SendCommandToChasis(TLT);
}

void PutOutTheLightCmd()                  // выключить освещение
{
  mode = PLT;
  SendCommandToChasis(PLT);
}

void ShineBrighterCmd()                   //усилить освещение
{
  mode = SHB;
  SendCommandToChasis(SHB);
}

void ShineDimmerCmd()                     //уменьшить освещение
{
  mode = SHD;
  SendCommandToChasis(SHD);
}

void SetSleepModeCmd()
{
  SendCommandToChasis(SLEEP);
}
