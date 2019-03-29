// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------------ SLAVE 101 ------------------------------------

Megnetometer :: Megnetometer(void)
{ 
  Wire.begin();
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x0B); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x01); // Set the Register
  Wire.endTransmission();
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x09); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x1D); // Set the Register
  Wire.endTransmission();
}

Megnetometer :: ~Megnetometer(void)
{
}

void Megnetometer :: GetRotationAngle(float *angleX, float *angleY, float *angleZ)
{  
  float startX = 0, startY = 0, startZ = 0;
  float endX = 0, endY = 0, endZ = 0;
  float dX = (float)9999.0, dY =(float)9999.0, dZ = (float)9999.0;
  GetMagnetometrData(&startX,&startY,&startZ);
  
  while(dX > 0.1 && dY > 0.1 && dZ > 0.1)
  {
    GetMagnetometrData(&endX,&endY,&endZ);
    dX = endX/(endX + endY + endZ);
    dY = endY/(endX + endY + endZ);
    dZ = endZ/(endX + endY + endZ); 
    
  }
    *angleX = abs(startX - endX);
    *angleY = abs(startY - endY);
    *angleZ = abs(startZ - endZ);
}

void Megnetometer :: GetMagnetometrData(float *x, float *y, float *z)
{ 
  byte counter = 0;
  long summX = 0, summY = 0, summZ = 0;
  int x1,y1,z1;
  
  while(counter < 51)
  {
    Wire.beginTransmission(addr);
    Wire.write(0x00); //start with register 3.
    Wire.endTransmission();
    Wire.requestFrom(addr, 6);
    if(6 <= Wire.available()) 
    {
      counter ++;
      x1 = Wire.read();               //MSB  x
      x1 |= Wire.read() << 8;         //LSB  x
      y1 = Wire.read();               //MSB  z
      y1 |= Wire.read() << 8;         //LSB z
      z1 = Wire.read();               //MSB y
      z1 |= Wire.read() << 8;         //LSB y
      summX += x1;
      summY += y1;
      summZ += z1;
    }
    delayMicroseconds(10000);
  }
  *x = (float)abs((summX/counter)/100);
  *y = (float)abs((summY/counter)/100);
  *z = (float)abs((summZ/counter)/100);
}
