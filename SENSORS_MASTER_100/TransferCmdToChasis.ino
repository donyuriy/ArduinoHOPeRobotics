// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------- MASTER(SENSORS) (Отправка комманд на другие контроллеры) ----------------------------------------

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

void Command :: SendCommandTo102(byte command)                 //отправка команд на устройство 102
{
  Wire.beginTransmission(SLAVE_DEVICE_102);
  Wire.write(command);
  Wire.endTransmission(true);
}


void Command :: ResetCmd()
{
  motionDirectionMode = STP;
  SendCommandToChasis(RST);
}

void Command :: MoveForwardCmd()                     //двигаться вперёд
{

  motionDirectionMode = FWD;
  SendCommandToChasis(FWD);
}

void Command :: MoveBackCmd()                        //двигаться назад
{
  motionDirectionMode = BWD;
  SendCommandToChasis(BWD);
}

void Command :: TurnRightCmd()                       //повернуть вправо
{
  motionDirectionMode = RGT;
  SendCommandToChasis(RGT);
}

void Command :: TurnLeftCmd()                        //повернуть влево
{
  motionDirectionMode = LFT;
  SendCommandToChasis(LFT);
}

void Command :: TurnBackCmd()                        // резвернуться на 180град.
{
  motionDirectionMode = TBK;
  SendCommandToChasis(TBK);
}

void Command :: StopTankCmd()                        // остановиться
{
  motionDirectionMode = STP;
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
  if (lightingMode != TLT)
  {
    lightingMode = TLT;
    SendCommandToChasis(TLT);
  }
}

void Command :: PutOutTheLightCmd()                  // выключить освещение
{
  if (lightingMode != PLT)
  {
    lightingMode = PLT;
    SendCommandToChasis(PLT);
  }
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

void Command :: GetMagnetometerValues()
{
  SendCommandToChasis(MGM);
}