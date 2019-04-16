// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------------ SLAVE 101 ------------------------------------

Magnetometer :: ~Magnetometer(void)
{
}

void Magnetometer :: GetRotationAngles()
{



}

void Magnetometer :: GetMagnetometrData()
{
  int x1;
  int y1;
  int z1;
  byte iterationCounter = 5;
  for (byte i = 0; i < iterationCounter; i++)
  {
    Wire.begin();
    Wire.beginTransmission(MAGNETOMETR_HMC5883_ADDRESS);
    Wire.write(MAGNETOMETR_HMC5883_MESASURING_COMMAND1);
    Wire.write(MAGNETOMETER_REGISTER_1);
    Wire.endTransmission();
    Wire.beginTransmission(MAGNETOMETR_HMC5883_ADDRESS);
    Wire.write(MAGNETOMETR_HMC5883_MESASURING_COMMAND2);
    Wire.write(MAGNETOMETER_REGISTER_2);
    Wire.endTransmission();
    Wire.beginTransmission(MAGNETOMETR_HMC5883_ADDRESS);
    Wire.write(MAGNETOMETER_REGISTER_3);
    Wire.endTransmission();
    Wire.requestFrom(MAGNETOMETR_HMC5883_ADDRESS, 6);

    if (Wire.available() >= 6)
    {
      x1 = Wire.read();               //MSB  x
      x1 |= Wire.read() << 8;         //LSB  x
      y1 = Wire.read();               //MSB  z
      y1 |= Wire.read() << 8;         //LSB z
      z1 = Wire.read();               //MSB y
      z1 |= Wire.read() << 8;         //LSB y

      axisXcurrent += x1;
      axisYcurrent += y1;
      axisZcurrent += z1;
    }
    else
    {
      i--;
    }
  }
  axisXcurrent /=  iterationCounter;
  axisYcurrent /= iterationCounter;
  axisZcurrent /= iterationCounter;
}

float Magnetometer :: filterX(float val) {
  xPc = xP + xvarProcess;
  xG = xPc / (xPc + xvarVolt);
  xP = (1 - xG) * xPc;
  xXp = xXe;
  xZp = xXp;
  xXe = xG * (val - xZp) + xXp;
  return (xXe);
}

float Magnetometer :: filterY(float val)
{
  yPc = yP + yvarProcess;
  yG = yPc / (yPc + yvarVolt);
  yP = (1 - yG) * yPc;
  yXp = yXe;
  yZp = yXp;
  yXe = yG * (val - xZp) + yXp;
  return (yXe);
}

float Magnetometer :: filterZ(float val)
{
  zPc = zP + zvarProcess;
  zG = zPc / (zPc + zvarVolt);
  zP = (1 - zG) * zPc;
  zXp = zXe;
  zZp = zXp;
  zXe = zG * (val - zZp) + zXp;
  return (zXe);
}
