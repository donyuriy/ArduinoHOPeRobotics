// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------------ SLAVE 101 ------------------------------------

Command :: Command(void)
{  
}

Command :: ~Command(void)
{  
}

void Command :: SendCommandToMaster(byte command)                 //отправка команд на Мастер
{  
  //Serial.print("I2C: "); Serial.println(command);
  Wire.beginTransmission(MASTER_DEVICE_SENSORS);
  Wire.write(command);
  Wire.endTransmission(true); 
}
