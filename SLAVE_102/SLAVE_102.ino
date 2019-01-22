// I2C-соединение (общие 5V & GND, соединение по A4 -> A4' , A5 -> A5' )
//------------------------------ SLAVE 102 ------------------------------------

#include <Wire.h>
#include <avr/sleep.h>

#define EMP 0                         // 0 - НИЧЕГО, НУЛЬ
#define SLEEP 2                       // 2 - отправить контроллеры в сон SLEEP
#define WUP 3                         // 3 - разбудить контроллеры WAKE UP
#define RST 9                         // 9 - комманда RESET
#define STP 10                        // 10 - СТОП
#define FWD 11                        // 11 - ВПЕРЕД
#define BWD 12                        // 12 - НАЗАД
#define SPU 15                        // 15 - УСКОРИТЬ
#define SDN 16                        // 16 - ЗАМЕДЛИТЬ
#define LFT 21                        // 21 - поворот НАЛЕВО
#define RGT 22                        // 22 - поворот НАПРАВО
#define TBK 23                        // 23 - РАЗВОРОТ на 180
#define TLT 31                        // 31 - ВКЛЮЧИТЬ СВЕТ
#define PLT 32                        // 32 - ВЫКЛЮЧИТЬ СВЕТ
#define SHB 33                        // 33 - сделать подсветку ЯРЧЕ
#define SHD 34                        // 34 - сделать подсветку ТУСКЛЕЕ
#define SUNON 41                      // 41 - режим СОЛНЕЧНОЙ БАТАРЕИ ВКЛЮЧЁН
#define SUNOFF 42                     // 42 - режим СОЛНЕЧНОЙ БАТАРЕИ ВЫКЛЮЧЕН 

#define THIS_SLAVE_DEVICE_NUMBER 102  // I2C-номер данного устройства

#define INTERRUPT_PIN 3               // пин для отработки прерывания Выход из Сна

byte mode = 0;                        //последний режим

void setup() 
{
  Wire.begin(THIS_SLAVE_DEVICE_NUMBER);
  Wire.onReceive(OnReceiveEventHandler);
}

void loop() 
{
  

}

void WakeUpNow()                      //обработка прерывания на порте D3
{}

void(*resetFunc) (void) = 0;

void OnReceiveEventHandler(int bytes)       //получение команды через I2C
{
  byte in_data = Wire.read();
  interrupts();
  ChooseAction(in_data);
}

void ChooseAction(byte in_data)       // выбор действия в зависимости от полученой команды
{  
  switch (in_data)
  {
    case EMP:                         
      mode = EMP;
      break;  
    case RST:                               
      ActionResetTankMode();
      resetFunc(); 
      break;  
    case SLEEP:
      ActionSetControlerToSleep();    
      break;      
    default:
      break;
  }
}

void ActionResetTankMode()                        
{
  mode = EMP;
  resetFunc();
}

void ActionSetControlerToSleep()                  // отправить устройство в сон
{    
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   
  sleep_enable();
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(1,WakeUpNow, LOW);
  mode = SLEEP;
  sleep_mode();
                //отслюда после пробуждения
  sleep_disable();
  detachInterrupt(1);
  interrupts();
  mode = EMP;
}
