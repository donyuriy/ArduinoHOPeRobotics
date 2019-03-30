// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------- MASTER ----------------------------------------

TestClass :: TestClass(void)
{  
}

TestClass :: ~TestClass(void)
{  
}

int TestClass :: RunMagnetometerTest()
{
  float angleX = 0;
  float angleY = 0;
  float angleZ = 0;
  
  mag.GetMagnetometrData(&angleX, &angleY, &angleZ);
  
  if(angleX == 0 && angleY == 0 && angleZ == 0)
  {
    return MEGNETOMETERDATAERROR;
  }
  return OK;
}
