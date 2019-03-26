// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------- MASTER (Отправка комманд на другие контроллеры) ----------------------------------------

Command :: Command(void)
{  
}

Command :: ~Command(void)
{  
}

void Command :: SendCommandToChasis(byte cmd)                 //отправка команд на шасси
{
  Wire.beginTransmission(SLAVE_DEVICE_CHASIS);
  Wire.write(cmd);
  Wire.endTransmission(true);  
  Wire.requestFrom(SLAVE_DEVICE_CHASIS, 2);
}

void Command :: ResetCmd()
{
  SendCommandToChasis(RST);
}

void Command :: MoveForwardCmd()                     //двигаться вперёд
{  
  SendCommandToChasis(FWD);
  extraMode = FWD;
}

void Command :: MoveBackCmd()                        //двигаться назад
{  
  SendCommandToChasis(BWD);
  extraMode = BWD;
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

void Command :: RunTest()                           //запустить самотестирование
{
  SendCommandToChasis(TEST);
}
