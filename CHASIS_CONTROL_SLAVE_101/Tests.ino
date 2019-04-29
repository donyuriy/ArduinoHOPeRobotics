// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------- SLAVE 101 Тесты----------------------------------

TestClass :: TestClass(void)
{
}

TestClass :: ~TestClass(void)
{
}

int TestClass :: RunMagnetometerTest()
{
  mag.GetRotationAngles();

  if (mag.horizontalAngle == 0 && mag.verticalAngle == 0 && mag.compas == 0)
  {
    return MEGNETOMETERDATAERROR;
  }
  return OK;
}
