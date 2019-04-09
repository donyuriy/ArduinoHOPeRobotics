// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------- MASTER ----------------------------------------

TestClass :: ~TestClass(void)
{  }

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
  int t1 = UsSensorsTestRun(); 
  if(t1 != OK)
  {
    HandleError(t1);
    return;
  }
  
  int t2 = PhotoSensorsTestRun();
  if(t2 != OK)
  {
    HandleError(t2);
    return;
  }
  
  int t3 = UsServosTestRun();
  if(t3 != OK)
  {
    HandleError(t3);
    return;
  }
  
  int t4 = SolarServosTestRun();
  if(t4 != OK)
  {
    HandleError(t4); 
    return; 
  }
  errorLevel = OK;
  ChasisModuleTestRun();
}

void TestClass :: HandleError(int error)
{
  //Serial.print("Error level: "); Serial.println(error);
    if(error != OK)
    {      
      errorLevel = error;
      switch(error)
      {
        case LEFTUSSENSORERROR:
        case RIGHTUSSENSORERROR:
        case CENTRALUSSENSORERROR:        
          Flasher(3);
          break;
        case PHOTOSENSORSOLAR1ERROR:
        case PHOTOSENSORSOLAR2ERROR:
        case PHOTOSENSORSOLAR3ERROR:
          Flasher(6);
          break;
        case SERVOUSSENSORERROR:
        case SERVOSOLARHORIZONTALERROR:
        case SERVOSOLARVERTICALERROR:
          Flasher(9); 
          break;
        default: break;       
      } 
    }
    else
    {
      digitalWrite(ERRORLED, LOW);
      errorLevel = OK;
    }  
}

void TestClass :: ChasisModuleTestRun()
{
  cmd.RunTest();
}

int TestClass :: UsSensorsTestRun()
{
  if(GetDistanceInCentimetersLeftSensor() <= 3.0)
  {
    return LEFTUSSENSORERROR;
  }
  if(GetDistanceInCentimetersRightSensor() <= 3.0)
  {
    return RIGHTUSSENSORERROR;
  }
  if(GetDistanceInCentimetersCentralSensor() <= 3.0)
  {
    return CENTRALUSSENSORERROR;
  }
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

  return OK;
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
  
  bc.RunServos(servoSunBatteryHorizontal.read(), MAX_SOLAR_HORIZONTAL_ANGLE, servoSunBatteryHorizontal);
  delay(delayTime);
  if(MAX_SOLAR_HORIZONTAL_ANGLE - servoSunBatteryHorizontal.read() > 3)
  {
    return SERVOSOLARHORIZONTALERROR;
  }
  
  bc.RunServos(servoSunBatteryVertical.read(), MIN_SOLAR_VERTICAL_ANGLE, servoSunBatteryVertical);
  delay(delayTime);
  if(servoSunBatteryVertical.read() - MIN_SOLAR_VERTICAL_ANGLE > 3)
  {
    return SERVOSOLARVERTICALERROR;
  }
  
  bc.RunServos(servoSunBatteryVertical.read(), MAX_SOLAR_VERTICAL_ANGLE, servoSunBatteryVertical);
  delay(delayTime);
  if(MAX_SOLAR_VERTICAL_ANGLE - servoSunBatteryVertical.read() > 3)
  {
    return SERVOSOLARVERTICALERROR;
  }
  return OK;
}
