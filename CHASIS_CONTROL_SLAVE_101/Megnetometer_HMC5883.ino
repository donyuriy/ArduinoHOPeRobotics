// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------------ SLAVE 101 ------------------------------------

Magnetometer :: ~Magnetometer(void)
{
}

void Magnetometer :: GetRotationAngles()
{
  byte correctionCoefficient = 12;
  if(axisDataError != 0)
  {
    GetDelta();
    angleX = deltaX * correctionCoefficient;
    angleY = deltaY * correctionCoefficient;
    angleZ = deltaZ * correctionCoefficient;
  }
  else
  {
    GetError();
    GetDelta();
    angleX = deltaX * correctionCoefficient;
    angleY = deltaY * correctionCoefficient;
    angleZ = deltaZ * correctionCoefficient;
  }
}

void Magnetometer :: GetMagnetometrData()
{
  int x1;
  int y1;
  int z1;
  byte iterationCounter = 15;
  operationTime = millis();
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
  operationTime = millis() - operationTime;
  //Serial.print("operationTime: ");Serial.println(operationTime);
}

void Magnetometer :: GetDelta()
{
  int newX, newY, newZ;
  int oldX, oldY, oldZ;

  GetMagnetometrData();
  oldX = axisXcurrent;
  oldY = axisYcurrent;
  oldZ = axisZcurrent;
  delay(50);

  GetMagnetometrData();
  newX = axisXcurrent;
  newY = axisYcurrent;
  newZ = axisZcurrent;

  deltaX = (newX - oldX) / (1 + axisDataError);
  deltaY = (newY - oldY) / (1 + axisDataError);
  deltaZ = (newZ - oldZ) / (1 + axisDataError);
}

void Magnetometer :: GetError()
{
  byte iterationCounter = 10;
  for (byte i = 0; i < iterationCounter; i++)
  {
    GetDelta();
    axisDataError += (abs(deltaX) + abs(deltaY) + abs(deltaZ)) / 3;
  }
  axisDataError = (float) axisDataError / iterationCounter;
  //Serial.print("axisDataError: "); Serial.println(axisDataError);
}
