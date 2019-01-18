// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------- MASTER (Отправка комманд на другие контроллеры) ----------------------------------------

void Command :: SendCommandToChasis(byte command)                 //отправка команд на шасси
{
  Wire.beginTransmission(SLAVE_DEVICE_CHASIS);
  Wire.write(command);
  Wire.endTransmission();
  Wire.requestFrom(SLAVE_DEVICE_CHASIS, 1);  
}

void Command :: ResetCmd()
{
  SendCommandToChasis(RST);
}

void Command :: MoveForwardCmd()                     //двигаться вперёд
{
  extraMode = FWD;
  SendCommandToChasis(FWD);
}

void Command :: MoveBackCmd()                        //двигаться назад
{
  extraMode = BWD;
  SendCommandToChasis(BWD);
}

void Command :: TurnRightCmd()                       //повернуть вправо
{
  SendCommandToChasis(RGT);
}

void Command :: TurnLeftCmd()                        //повернуть влево
{
  SendCommandToChasis(LFT);
}

void Command :: TurnBackCmd()                        // резвернуться на 180град.
{
  SendCommandToChasis(TBK);
}

void Command :: StopTankCmd()                        // остановиться
{
  extraMode = STP;
  SendCommandToChasis(STP);
}

void Command :: SpeedUpCmd()                         // ускориться
{
  SendCommandToChasis(SPU);
}

void Command :: SlowDownCmd()                        // замедлиться
{
  SendCommandToChasis(SDN);
}

void Command :: TurnOnTheLightCmd()                  //включить освещение
{
  SendCommandToChasis(TLT);
}

void Command :: PutOutTheLightCmd()                  // выключить освещение
{
  SendCommandToChasis(PLT);
}

void Command :: ShineBrighterCmd()                   //усилить освещение
{
  SendCommandToChasis(SHB);
}

void Command :: ShineDimmerCmd()                     //уменьшить освещение
{
  SendCommandToChasis(SHD);
}

void Command :: SetSleepModeCmd()                    // установить режим сна для шасси
{
  SendCommandToChasis(SLEEP);
}
