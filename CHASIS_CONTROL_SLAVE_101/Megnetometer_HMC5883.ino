// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------------ SLAVE 101 ------------------------------------

Magnetometer :: Magnetometer(void)
{
    float AxisXcurrent = 0;
    float AxisYcurrent = 0;
    float AxisZcurrent = 0;
    float DeltaX = 0;
    float DeltaY = 0;
    float DeltaZ = 0;
}

Magnetometer :: ~Magnetometer(void)
{
}

void Magnetometer :: GetMagnetometrData()
{
  int x1;
  int y1;
  int z1;
  
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

    AxisXcurrent = x1;
    AxisYcurrent = y1;
    AxisZcurrent = z1;     
  }
}

void Magnetometer :: GetRotationAngle()
{
  int newX;
  int newY;
  int newZ;
  int oldX;
  int oldY;
  int oldZ;
  
  GetMagnetometrData();
  oldX = AxisXcurrent;
  oldY = AxisXcurrent;
  oldZ = AxisZcurrent;
  delay(10);
  GetMagnetometrData();
  newX = AxisXcurrent;
  newY = AxisXcurrent;
  newZ = AxisZcurrent;

  DeltaX = (float)(newX - oldX)/100;
  DeltaY = (float)(newY - oldY)/100;
  DeltaZ = (float)(newZ - oldZ)/100;
}
