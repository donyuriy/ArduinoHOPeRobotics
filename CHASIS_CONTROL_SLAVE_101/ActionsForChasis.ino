// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------------ SLAVE 101 ------------------------------------


void ChasisActions :: ActionResetTankMode()                        
{
  mode = EMP;
  ActionStopTank();
  ActionPutOutTheLight();
}

void ChasisActions :: ActionMoveTankForward()       
{
  tankDirection = motorFORWARD;
  motor->chooseMotor(LEFT_MOTOR, tankDirection, tankSpeed);
  motor->chooseMotor(RIGHT_MOTOR, tankDirection, tankSpeed - engineTorqueRatio);  
  mode = FWD;
}

void ChasisActions :: ActionMoveTankBackward()    
{
  tankDirection = motorBACKWARD;
  motor->chooseMotor(LEFT_MOTOR, tankDirection, tankSpeed);
  motor->chooseMotor(RIGHT_MOTOR, tankDirection, tankSpeed - engineTorqueRatio);
  mode = BWD;
}

void ChasisActions :: ActionStopTank()                              
{
  motor->chooseMotor(LEFT_MOTOR, motorRELEASE, 0);
  motor->chooseMotor(RIGHT_MOTOR, motorRELEASE, 0);
  tankDirection = 0;
  mode = STP;
}

void ChasisActions :: ActionTurnTankLeft()
{
  while(millis() - dTtemp < DALAY_TIME)
  {
    motor->chooseMotor(LEFT_MOTOR, motorBACKWARD, tankSpeed + 20);
    motor->chooseMotor(RIGHT_MOTOR, motorFORWARD, tankSpeed + 20);
  }  
  if(mode == FWD)
  {
    ActionMoveTankForward();
  }
  else if(mode == BWD)
  {
    ActionMoveTankBackward();
  }
  else 
  {
    ActionStopTank();
  }  
}

void ChasisActions :: ActionTurnTankRight()           
{
  while(millis() - dTtemp < DALAY_TIME)
  {
    motor->chooseMotor(LEFT_MOTOR, motorFORWARD, tankSpeed + 20);
    motor->chooseMotor(RIGHT_MOTOR, motorBACKWARD, tankSpeed + 20);
  }
  if(mode == FWD)
  {
    ActionMoveTankForward();
  }
  else if(mode == BWD)
  {
    ActionMoveTankBackward();
  }
  else 
  { 
    ActionStopTank();
  } 
}

void ChasisActions :: ActionTurnTankBack()                           
{
  while(millis() - dTtemp < DALAY_TIME*2)
  {
    motor->chooseMotor(LEFT_MOTOR, motorFORWARD, tankSpeed + 20);
    motor->chooseMotor(RIGHT_MOTOR, motorBACKWARD, tankSpeed + 20);
  }  
  if(mode == FWD)
  {
    ActionMoveTankForward();
  }
  else if(mode == BWD)
  {
    ActionMoveTankBackward();
  }
  else 
  {
    ActionStopTank(); 
  } 
}

void ChasisActions :: ActionSpeedUpTank()     
{
  tankSpeed += 5;
  if(tankSpeed > 250)tankSpeed = 250;  
    if (mode == FWD)
    {
      ActionMoveTankForward();
    }
    else if(mode != BWD)
    {
      ActionMoveTankBackward();
    }
}

void ChasisActions :: ActionSlowDownTank()        
{
  tankSpeed -= 5;
  if(tankSpeed < 150)tankSpeed = 150;
   if (mode == FWD)
    {
      ActionMoveTankForward();
    }
    else if(mode != BWD)
    {
      ActionMoveTankBackward();
    }
}

void ChasisActions :: ActionTurnOnTheLight()                     
{
  if(lightBrightness == 0)
  {
    lightBrightness = 10;  
    motor->chooseMotor(LED, motorFORWARD, lightBrightness);    
    mode = TLT;
  }
}

void ChasisActions :: ActionPutOutTheLight()    
{
  if(lightBrightness > 0)
  {
    lightBrightness = 0;
    motor->chooseMotor(LED,motorRELEASE,lightBrightness);    
    mode = PLT;
  }
}

void ChasisActions :: ActionShineBrighter()                  
{
  lightBrightness += 3;
  if(lightBrightness > 30)lightBrightness = 30;
  motor->chooseMotor(LED, motorFORWARD, lightBrightness);
}

void ChasisActions :: ActionShineDimmer()                   
{
  lightBrightness -= 3;
  if(lightBrightness <= 0)
  {
    ActionPutOutTheLight();
  }
  else
  {
    motor->chooseMotor(LED, motorFORWARD, lightBrightness);
  }
}
