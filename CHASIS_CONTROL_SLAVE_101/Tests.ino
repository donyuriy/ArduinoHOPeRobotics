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
  mag.GetRotationAngles();

  if (mag.angleX == 0 && mag.angleY == 0 && mag.angleZ == 0)
  {
    return MEGNETOMETERDATAERROR;
  }
  return OK;
}
