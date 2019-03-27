// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------- MASTER ----------------------------------------

TestClass :: TestClass(void)
{  
}

TestClass :: ~TestClass(void)
{  
}

void TestClass :: Flasher(byte count)
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

void TestClass :: RunSelfTest()
{
  if(errorLevel == 0 || errorLevel == OK)
  {
    errorLevel = UsSensorsTestRun();
    errorLevel = PhotoSensorsTestRun();
    errorLevel = UsServosTestRun();
    errorLevel = SolarServosTestRun();
  }
  
  if(errorLevel != OK)
  {
   Flasher(3);              
  }  
  else
  {
    digitalWrite(ERRORLED, LOW);
  }
}

int TestClass :: UsSensorsTestRun()
{
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
  return OK;
}

int TestClass :: PhotoSensorsTestRun()
{
   int phMax = 1023;
   int phMin = 0; 
  
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
  return OK;
}

int TestClass :: UsServosTestRun()
{
  byte a = 10;
  byte b = 170;
  byte c = 100;
  int delayTime = 500;
  
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
}

int TestClass :: SolarServosTestRun()
{
  int delayTime = 500;
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
