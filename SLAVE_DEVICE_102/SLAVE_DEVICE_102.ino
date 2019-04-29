// I2C-соединение (общие 5V & GND, соединение по A4 -> SDA , A5 -> SCL )
//------------------------- SLAVE 102(Additional functionality) ----------------------------------------

//Libraries
#include <Wire.h>

//Commands for I2C interface
#define EMP 100                        // 100 - НИЧЕГО, НУЛЬ
#define SLEEP 102                      // 102 - отправить контроллеры в сон SLEEP
#define WUP 103                        // 103 - разбудить контроллеры WAKE UP
#define TEST 104                       // 104 - тест всех датчиков и приводов
#define RST 109                        // 19 - комманда RESET
#define STP 110                        // 110 - СТОП
#define FWD 111                        // 111 - ВПЕРЕД
#define BWD 112                        // 112 - НАЗАД
#define SPU 115                        // 115 - УСКОРИТЬ
#define SDN 116                        // 116 - ЗАМЕДЛИТЬ
#define LFT 121                        // 121 - поворот НАЛЕВО
#define RGT 122                        // 122 - поворот НАПРАВО
#define TBK 123                        // 123 - РАЗВОРОТ на 180
#define TLT 131                        // 131 - ВКЛЮЧИТЬ СВЕТ
#define PLT 132                        // 132 - ВЫКЛЮЧИТЬ СВЕТ
#define SHB 133                        // 133 - сделать подсветку ЯРЧЕ
#define SHD 134                        // 134 - сделать подсветку ТУСКЛЕЕ
#define SUNON 141                       // 141 - режим СОЛНЕЧНОЙ БАТАРЕИ ВКЛЮЧЁН         глобальный
#define SUNOFF 142                      // 142 - режим СОЛНЕЧНОЙ БАТАРЕИ ВЫКЛЮЧЕН        глобальный 
#define CHASIS_ERR 251                  // 251 -сообщение об ошибке на шасси

//System variables
#define MASTER_DEVICE_SENSORS 0x64                        // I2C-номер устройства Master (Sensors)
#define SLAVE_DEVICE_CHASIS 0x65                          // I2C-номер устройства шасси
#define SLAVE_DEVICE_102 0x66                             // I2C-номер этого устройства (102)

class Command
{
  public:
    Command();
    ~Command();
    void SendCommandToChasis(byte command);
    void SendCommandToMaster(byte command);
};

Command cmd;

void setup()
{
  //Serial.begin(9600);
  Wire.begin(SLAVE_DEVICE_102);
  Wire.onReceive(OnReceiveEventHandler);
  delay(100);
}

void loop()
{}

void OnReceiveEventHandler(int howMany)
{ 
  if(Wire.available() > 0)
  {
    byte in_data = Wire.read();
    interrupts();
    ChooseAction(in_data);
  }
}

void ChooseAction(byte in_data)
{
  //Serial.print("Master in_data: "); Serial.println(in_data);
  switch(in_data)
  {    
    default:
      break;
  }
}
