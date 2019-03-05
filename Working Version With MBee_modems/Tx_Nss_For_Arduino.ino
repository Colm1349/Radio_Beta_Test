/**333
   Этот файл является частью библиотеки MBee-Arduino.

   MBee-Arduino является бесплатным программным обеспечением.
   Подробная информация о лицензиях находится в файле mbee.h.

   \author </i> von Boduen. Special thanx to Andrew Rapp.
*/

// Version 0.9

#include <MBee.h>
#include <SoftwareSerial.h>

#define PC_Serial nss
#define MBee_Serial Serial
#define Stop 0
#define Forward 5
#define Backward 2
#define Corrupted_Command 10
#define EmergencyStopCode 170
#define StopButtonPin 7

uint8_t ssRX = 8;
uint8_t ssTX = 9;
SoftwareSerial nss(ssRX, ssTX);
SerialStar mbee = SerialStar();

uint16_t remoteAddress = 0x0001; //Адрес модема, которому будут передаваться данные.

char testString[] = "Hello world!";
uint8_t testArray [] = {200, 100, 88};

TxRequest tx = TxRequest(); //Пакет с данными для отправки удаленному модему. Для экономии памяти будем использовать один и тот же объект для отправки всех пакетов.
TxStatusResponse txStatus = TxStatusResponse(); //Локальный ответ со статусом команды.
RxAcknowledgeResponse remoteRxStatus = RxAcknowledgeResponse(); //Пакет с подтверждением от удаленного модема получение данных.

int errorLed = 56;

//int statusLed = LED_BUILTIN; //Используется встроенный в Вашу плату Arduino светодиод.
int statusLed = 13;

int Green_Led = 11;
int Red_Led = 12;
int ForwardPin = 2;
int BackwardPin = 3;
int CommandForRaper = 0; // 0 == stop
bool lastButton = false;
bool currentButton = false;

void setup()
{
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(Green_Led, OUTPUT);
  pinMode(Red_Led, OUTPUT);
  pinMode(ForwardPin , INPUT);
  pinMode(BackwardPin, INPUT);
  PC_Serial.begin(115200);
  MBee_Serial.begin(115200);
  mbee.begin(MBee_Serial);
  delay(500); //Задержка не обязательна и вставлена для удобства работы с терминальной программой.
}

void loop()
{
  static int number_of_packet = 0;
  PC_Serial.print("Sending...[");
  PC_Serial.print(number_of_packet);
  PC_Serial.println("]");
  PC_Serial.println();
  tx.setRemoteAddress(remoteAddress); //Устанавливаем адрес удаленного модема.
  tx.setPayload((uint8_t*)testArray); //Устанавливаем указатель на тестовый массив
  tx.setPayloadLength(sizeof(testArray));  //Устанавливаем длину поля данных
  //Read command
  ReadCommandFromSwither();
  //Write our command into "testArray" for send.
  //  if (CommandForRaper != 0)
  //  {
  testArray[0] = number_of_packet;
  testArray[1] = CommandForRaper;
  //  }
  currentButton = debounce(lastButton);
  if ( currentButton == true )
  {
    PC_Serial.println("STOP THIS");
    delay(10);
    testArray[2] = EmergencyStopCode;
  }
  else
  {
    PC_Serial.println("All HOPE IS OK");
//    delay(100);
    testArray[2] = 88;
  }
  sendData(); // sending
  // +1 counter of packet
  number_of_packet++;
  //testArray [0] = number_of_packet;
  delay(10);
}

void ReadCommandFromSwither()
{
  //Check  a move command from  switch(remote controller)
  bool Rush = digitalRead(ForwardPin);
  bool Back = digitalRead(BackwardPin);
  // Forward command
  if (Rush == true & Back == false)
  {
    CommandForRaper = Forward;
    digitalWrite(Red_Led,  LOW);
    digitalWrite(Green_Led, HIGH);
    PC_Serial.println("Command RUSH ");
  }
  // Backward command
  if (Rush == false & Back == true)
  {
    CommandForRaper = Backward;
    digitalWrite(Red_Led,  HIGH);
    digitalWrite(Green_Led, LOW);
    PC_Serial.println("Command BACK ");
  }
  // Stop command
  if (Rush == false & Back == false)
  {
    CommandForRaper = Stop;
    digitalWrite(Red_Led,  LOW);
    digitalWrite(Green_Led, LOW);
    PC_Serial.println("Command STOP !!! ");
  }
  // Error
  if (Rush == true & Back == true)
  {
    PC_Serial.println();
    PC_Serial.println("I take Error from \"switcher\" , =( ");
    PC_Serial.println();
    CommandForRaper = Stop;
    digitalWrite(Red_Led,  HIGH);
    digitalWrite(Green_Led, HIGH);
    digitalWrite(statusLed, HIGH);
    PC_Serial.println(" NO Command , we have a ERROR ((( ");
  }
  PC_Serial.print("CommandForRaper == ");
  PC_Serial.println(CommandForRaper);
}


void sendData()
{
  mbee.send(tx);
  if (tx.getFrameId()) //Проверяем, не заблокировано ли локальное подтверждение отправки.
    getLocalResponse(50);
  if ((tx.getFrameId() == 0) || (txStatus.isSuccess() && tx.getSleepingDevice() == false)) //Ждем ответного пакета от удаленного модема только если локальный ответ выключен или пакет отправлен в эфир и не предназначается спящему модему.
    getRemoteResponse(100);
}

void getLocalResponse(uint16_t timeout)
{
  if (mbee.readPacket(timeout)) //Ждем ответа со статусом команды.
  {
    if (mbee.getResponse().getApiId() == TRANSMIT_STATUS_API_FRAME) //Проверяем, является ли принятый пакет локальным ответом на AT-команду удаленному модему.
    {
      mbee.getResponse().getTxStatusResponse(txStatus);
      if (txStatus.isSuccess())
        flashLed(statusLed, 1, 200); //Порядок. Пакет ушел в эфир.
      else if (txStatus.getStatus() == TX_FAILURE_COMMAND_STATUS)
        flashLed(errorLed, 1, 200); //Пакет в эфир не ушел вследствие занятости канала.
      else if (txStatus.getStatus() == ERROR_COMMAND_STATUS)
        flashLed(errorLed, 1, 1000); //Недостаточно памяти для размещения пакета в буфере на передачу.
    }
  }
}

void getRemoteResponse(uint16_t timeout)
{
  if (mbee.readPacket(timeout)) //Ждем подтверждения получения данных от удаленного модуля.
  {
    if (mbee.getResponse().getApiId() == REMOTE_ACKNOWLEDGE_API_FRAME) //Является ли полученный фрейм подтверждением доставки.
    {
      mbee.getResponse().getRxAcknowledgeResponse(remoteRxStatus);
      if (remoteRxStatus.getFrameId() == tx.getFrameId()) //Проверяем,совпадает ли идентификатор фрейма в полученном подтверждении с идентификатором отправленного пакета.
        flashLed(statusLed, 2, 200); //Подтверждение получено именно на отпправленный только что пакет. Все прекрасно.
      else
        flashLed(errorLed, 2, 200); //Идентификатор пакета в принятом подтверждении не совпал с идентификтором отправленного пакета.
    }
    else
      flashLed(errorLed, 3, 200); //Принятый пакет не является пакетом подтверждения доставки.
  }
  else
  {
    if (mbee.getResponse().isError())
      flashLed(errorLed, 4, 200); //В процессе разбора принятого пакета произошли ошибки.
    else
      flashLed(errorLed, 1, 2000); //Нет ответа от удаленного модема.
  }
}

void flashLed(int pin, int times, int wait)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(pin, HIGH);
    delay(1);
    //    delay(wait);
    digitalWrite(pin, LOW);
    if (i + 1 < times)
      //      delay(wait);
      delay(1);
  }
}

bool EmergencyStopButtonIsPressed()
{
  bool answer;
  currentButton = debounce(lastButton);
  if (lastButton == LOW && currentButton == HIGH)
  {
    answer = true;
    PC_Serial.println("Emergency Stop  == TRUE ");
    delay(500);
  }
  lastButton = currentButton;
  answer = false;
  return answer;
}

boolean debounce(boolean last)
{
  boolean current = digitalRead(StopButtonPin);
  if (last != current)
  {
    delay(5);
    Serial.println("last != current");
    current = digitalRead(StopButtonPin);
  }
  return current;
}
