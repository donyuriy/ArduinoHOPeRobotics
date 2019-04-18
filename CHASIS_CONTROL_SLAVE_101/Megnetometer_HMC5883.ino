// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------------ SLAVE 101 Магнитометр-------------------------

Magnetometer :: ~Magnetometer(void)
{
}

void Magnetometer :: GetRotationAngles()
{
  // X - относительная, показывает отклонение от стартовой точки в ГОРИЗОНТАЛЬНОЙ плоскости
  // Z - относительная, показывает отклонение от стартовой точки в ВЕРТИКАЛЬНОЙ плоскости
  // Y - абсолютная, стационарно показывает направление по сторонам света (ЮГ => min, СЕВЕР => max ЗАПАД != ВОСТОК)
 
  GetMagnetometerData();
  float x = axisXcurrent;
  float y = axisYcurrent;
  float z = axisZcurrent;
  delay(100);

  GetMagnetometerData();
  x = abs(x) - abs(axisXcurrent);
  y = abs(y) - abs(axisYcurrent);
  z = abs(z) - abs(axisZcurrent);

  if (2 > abs(x)) {
    verticalAngle = 0;
  }
  else {
    verticalAngle = x;
  }

  if (2 > abs(z)) {
    horizontalAngle = 0;
  }
  else {
    horizontalAngle = z;
  }

  compas = y;
}

void Magnetometer :: GetMagnetometerData()
{
  int x;
  int y;
  int z;
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
    x = Wire.read();               //MSB  x
    x |= Wire.read() << 8;         //LSB  x
    y = Wire.read();               //MSB  z
    y |= Wire.read() << 8;         //LSB z
    z = Wire.read();               //MSB y
    z |= Wire.read() << 8;         //LSB y

    float x1 = (float)x;
    float y1 = (float)y;
    float z1 = (float)z;

    axisXcurrent = filterX(x1);
    axisYcurrent = filterY(y1);
    axisZcurrent = filterZ(z1);
  }
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
