// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------- MASTER (Работа с питанием -> батарея, солнечная панель) ----------------------------------------

  
  float BatteryClass :: GetBattaryVoltage()                                     //получить усредненное значение напряжения на батарее
  {
    float data = 0;
    byte avarage = 25;
    for (byte i = 0; i < avarage; i ++)
    {
      data += analogRead(VOLTMETER_SENSOR_PIN) * 5.0 / 1024.0;
    }
    return (float)(data / avarage);
  }
  
  bool BatteryClass :: IsBatteryPowerNormal()                                     // проверка достаточности заряда батареи
  {
    if (GetBattaryVoltage() > HIGH_BATTERY_CHARGE) return true;
    else return false;
  }
  
  void BatteryClass :: CheckBatteryVoltage()                                      //проверить заряд БАТАРЕИ
  {
    //Serial.println("CheckBatteryVoltage");
    if (GetBattaryVoltage() <= LOWEST_BATTERY_CHARGE)
    {
      sendCommand->StopTankCmd();
      sendCommand->SetSleepModeCmd();      
    }
    if (GetBattaryVoltage() <= LOW_BATTERY_CHARGE)
    { 
      sendCommand->StopTankCmd();
      if (globalMode != SUNON) globalMode = SUNON;
      if (GetPhotoSensorData(1) < MINIMAL_BRIGHTNESS_LEVEL_FOR_SOLAR_BATTERY &&
          GetPhotoSensorData(2) < MINIMAL_BRIGHTNESS_LEVEL_FOR_SOLAR_BATTERY &&
          GetPhotoSensorData(3) < MINIMAL_BRIGHTNESS_LEVEL_FOR_SOLAR_BATTERY)
      {
        ActionSolarBatteryOn();
      }
    }
    else if (GetBattaryVoltage() > HIGH_BATTERY_CHARGE)
    {
      ActionSolarBatteryOff();
    }
  }
  
  void BatteryClass :: ActionSolarBatteryOn()                                     //включить Солнечную батарею
  { 
    //Serial.println("ActionSolarBatteryOn");
    verticalSunBattery_angle = servoSunBatteryVertical.read();
    horizontalSunBattery_angle = servoSunBatteryHorizontal.read();
    SearchHorizontalSolar();    
    SearchVerticalSolar();  
  }
  
  void BatteryClass :: SearchVerticalSolar()                                              // поиск максимальной освещенности в вертикальной плоскости
  {    
    int photoSensor1 = GetPhotoSensorData(1);
    int photoSensor2 = GetPhotoSensorData(2);        
   
    while ((photoSensor1 - photoSensor2 < -photosensorDefference &&  verticalSunBattery_angle < MAX_SOLAR_VERTICAL_ANGLE)||
          (photoSensor1 - photoSensor2 > photosensorDefference && MIN_SOLAR_VERTICAL_ANGLE < verticalSunBattery_angle ))
    {
      if (photoSensor1 - photoSensor2 < -photosensorDefference)
      {        
        verticalSunBattery_angle++;
        if (verticalSunBattery_angle < MIN_SOLAR_VERTICAL_ANGLE)verticalSunBattery_angle = MIN_SOLAR_VERTICAL_ANGLE;
        else if (verticalSunBattery_angle > MAX_SOLAR_VERTICAL_ANGLE)verticalSunBattery_angle = MAX_SOLAR_VERTICAL_ANGLE;
        RunServos(servoSunBatteryVertical.read(), verticalSunBattery_angle, servoSunBatteryVertical);
        photoSensor1 = GetPhotoSensorData(1);
        photoSensor2 = GetPhotoSensorData(2);
      }
      else if (photoSensor1 - photoSensor2 > photosensorDefference)
      {
        verticalSunBattery_angle--;
        if (verticalSunBattery_angle < MIN_SOLAR_VERTICAL_ANGLE)verticalSunBattery_angle = MIN_SOLAR_VERTICAL_ANGLE;
        else if (verticalSunBattery_angle > MAX_SOLAR_VERTICAL_ANGLE)verticalSunBattery_angle = MAX_SOLAR_VERTICAL_ANGLE;
        RunServos(servoSunBatteryVertical.read(), verticalSunBattery_angle, servoSunBatteryVertical);
        photoSensor1 = GetPhotoSensorData(1);
        photoSensor2 = GetPhotoSensorData(2);
      }
    }
  }
  
  void BatteryClass :: SearchHorizontalSolar()                                                // поиск максимальной освещенности в горизонтальной плоскости
  {       
    float correctionIndex = 220;
    int photoSensor3 = GetPhotoSensorData(3);
    int avarageSensors = (GetPhotoSensorData(1) + GetPhotoSensorData(2) + correctionIndex)/2;

    while ((photoSensor3 - avarageSensors < -photosensorDefference &&  horizontalSunBattery_angle < MAX_SOLAR_HORIZONTAL_ANGLE)||
            (photoSensor3 - avarageSensors > photosensorDefference && MIN_SOLAR_HORIZONTAL_ANGLE < horizontalSunBattery_angle ))
    { 
      if (photoSensor3 - avarageSensors < -photosensorDefference)
      {
        horizontalSunBattery_angle++;
        if (horizontalSunBattery_angle < MIN_SOLAR_HORIZONTAL_ANGLE)horizontalSunBattery_angle = MIN_SOLAR_HORIZONTAL_ANGLE;
        else if (horizontalSunBattery_angle > MAX_SOLAR_HORIZONTAL_ANGLE)horizontalSunBattery_angle = MAX_SOLAR_HORIZONTAL_ANGLE;
        RunServos(servoSunBatteryHorizontal.read(), horizontalSunBattery_angle, servoSunBatteryHorizontal);
        photoSensor3 = GetPhotoSensorData(3);
        avarageSensors = (GetPhotoSensorData(1) + GetPhotoSensorData(2) + correctionIndex)/2;
      }
      else if (photoSensor3 - avarageSensors > photosensorDefference)
      {
        horizontalSunBattery_angle--;
        if (horizontalSunBattery_angle < MIN_SOLAR_HORIZONTAL_ANGLE)horizontalSunBattery_angle = MIN_SOLAR_HORIZONTAL_ANGLE;
        else if (horizontalSunBattery_angle > MAX_SOLAR_HORIZONTAL_ANGLE)horizontalSunBattery_angle = MAX_SOLAR_HORIZONTAL_ANGLE;
        RunServos(servoSunBatteryHorizontal.read(), horizontalSunBattery_angle, servoSunBatteryHorizontal);
        photoSensor3 = GetPhotoSensorData(3);
        avarageSensors = (GetPhotoSensorData(1) + GetPhotoSensorData(2) + correctionIndex)/2;
      }
    }
  }
  
  void BatteryClass :: SetUpSolarBattery(Servo servoSunBatteryVertical, Servo servoSunBatteryHorizontal)         // установка солнечной панели в положение по умолчанию
  {
    //Serial.println("SetUpSolarBattery");
    byte X = MIN_SOLAR_VERTICAL_ANGLE;
    byte Y = MIN_SOLAR_HORIZONTAL_ANGLE + 10;
    if (servoSunBatteryVertical.read() -  X > 3 || servoSunBatteryVertical.read() -  X < -3)
    {
      RunServos(servoSunBatteryVertical.read(), X, servoSunBatteryVertical);
    }
    if (servoSunBatteryHorizontal.read() -  Y > 3 || servoSunBatteryHorizontal.read() -  Y < -3)
    {
      RunServos(servoSunBatteryHorizontal.read(), Y, servoSunBatteryHorizontal);
    }    
  }
  
  void BatteryClass :: ActionSolarBatteryOff()     //отключить Солнечную батарею
  {
    //Serial.println("ActionSolarBatteryOff");
    SetUpSolarBattery(servoSunBatteryVertical, servoSunBatteryHorizontal);
    WakeUpShields();
    globalMode = EMP;
    extraMode = STP;
  }
  
  float BatteryClass :: GetPhotoSensorData(byte sensorID)     // получить показания с аналогового фотосенсора № 1 или 2
  {
    int data = 0;
    byte avarage = 20;
    switch (sensorID)
    {
      case 1:
        for (byte i = 0; i < avarage; i++)
        {
          data += analogRead(SOLAR_SENSOR_PIN_1);
        }
        break;
      case 2:
        for (byte i = 0; i < avarage; i++)
        {
          data += analogRead(SOLAR_SENSOR_PIN_2);
        }
        break;
      case 3: 
        for (byte i = 0; i < avarage; i++)
        {
          data += analogRead(SOLAR_SENSOR_PIN_3);
        }
        break;
      default:
        break;      
    }
    return (float)(data / avarage);
  }

void BatteryClass :: RunServos(byte ServoStartAngle, byte ServoFinishAngle, Servo servo)                     // замедленный поворот сервоприводов
{
  int delayInterval = 7;
  if (ServoStartAngle < ServoFinishAngle)
  {
    for (byte i = ServoStartAngle; i < ServoFinishAngle; i++)
    {
      servo.write(i);
      delay(delayInterval);
    }
  }
  else
  {
    for (byte i = ServoStartAngle; i > ServoFinishAngle; i--)
    {
      servo.write(i);
      delay(delayInterval);
    }
  }
}
