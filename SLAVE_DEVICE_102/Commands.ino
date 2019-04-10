// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------- SLAVE 102(Additional functionality) ----------------------------------------

Command :: Command(void)
{  
}

Command :: ~Command(void)
{  
}

void Command :: SendCommandToChasis(byte command)                 //отправка команд на шасси
{  
  Wire.beginTransmission(SLAVE_DEVICE_CHASIS);
  Wire.write(command);
  Wire.endTransmission(true);  
}

void Command :: SendCommandToMaster(byte command)                 //отправка команд на Мастер
{  
  //Serial.print("I2C: "); Serial.println(command);
  Wire.beginTransmission(MASTER_DEVICE_SENSORS);
  Wire.write(command);
  Wire.endTransmission(true); 
}
