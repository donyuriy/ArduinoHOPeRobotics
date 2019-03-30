// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------------ SLAVE 101 ------------------------------------

Magnetometer :: Magnetometer(void)
{ 
  
}

Magnetometer :: ~Magnetometer(void)
{
}

void Magnetometer :: GetMagnetometrData(float *x, float *y, float *z)
{ 
  int x1,y1,z1;
  Wire.begin();
  Wire.beginTransmission(addr);   //start talking
  Wire.write(0x0B);               // Tell the HMC5883 to Continuously Measure
  Wire.write(0x01);               // Set the Register
  Wire.endTransmission();
  Wire.beginTransmission(addr);   //start talking
  Wire.write(0x09);               // Tell the HMC5883 to Continuously Measure
  Wire.write(0x1D);               // Set the Register
  Wire.endTransmission();
    
  Wire.beginTransmission(addr);
  Wire.write(0x00); //start with register 3.
  Wire.endTransmission();
  Wire.requestFrom(addr, 6);
    
  if(Wire.available() >= 6) 
  {
    x1 = Wire.read();               //MSB  x
    x1 |= Wire.read() << 8;         //LSB  x
    y1 = Wire.read();               //MSB  z
    y1 |= Wire.read() << 8;         //LSB z
    z1 = Wire.read();               //MSB y
    z1 |= Wire.read() << 8;         //LSB y
    
    *x = (float)abs(x1);
    *y = (float)abs(y1);
    *z += (float)abs(z1);
  }     
}
