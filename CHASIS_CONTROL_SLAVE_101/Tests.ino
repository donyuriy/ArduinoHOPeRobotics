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
  mag.GetMagnetometrData();

  if (mag.AxisXcurrent == 0 && mag.AxisYcurrent == 0 && mag.AxisZcurrent == 0)
  {
    return MEGNETOMETERDATAERROR;
  }
  return OK;
}
