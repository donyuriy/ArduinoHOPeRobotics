// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------------ SLAVE 101 Отправка команд ----------------------------

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

void Command :: SendCommandTo102(byte command)                 //отправка команд на устройство 102
{  
  Wire.beginTransmission(SLAVE_DEVICE_102);
  Wire.write(command);
  Wire.endTransmission(true);  
}

void Command :: SendOkToMaster()
{
  SendCommandToMaster(OK);
}

void Command :: SendErrorToMaster()
{
  SendCommandToMaster(MEGNETOMETERDATAERROR);
}
