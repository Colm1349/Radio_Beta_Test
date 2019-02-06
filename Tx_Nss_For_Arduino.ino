/**
   Этот файл является частью библиотеки MBee-Arduino.

   MBee-Arduino является бесплатным программным обеспечением.
   Подробная информация о лицензиях находится в файле mbee.h.

   \author </i> von Boduen. Special thanx to Andrew Rapp.
*/

#include <MBee.h>
#include <SoftwareSerial.h>

/**
    Скетч предназначен для демонстрации передачи пакетов с данными удаленному модему.
    Передающий и принимающий модемы работают под управлением программного обеспечения SerialStar
    для модулей MBee-868-x.0.
    Действия, производимые скетчем подробно описаны в комментариях к соответствующим строкам.
    Потребуются 2 модуля MBee-868-x.0. Первый модуль соедининяется с платой Arduino c помощью
    XBee-shield или любого другого совместимого устройств. Если доступного шилда нет, то возможно
    соединение Arduino и модуля с помощью проводов.
    ВНИМАНИЕ!!! Модуль MBee-868-x.0 имеет номинальное значение напряжения питания 3,3В. Если Ваша
    плата Arduino имеет выходы с логическими уровнями 5В, то необходимо предусмотреть делитель
    напряжения между выходом TX Arduino и входом RX модуля (вывод №3 для всех моделей). К выводу TX
    Arduino подключается резистор 2К, с которым соединен резистор 1К, второй вывод последнего
    сажается на землю. Точка соединения резисторов соединяется с выводом №3 модуля.
    Вывод №2 модуля (TX), подключается ко входу RX Arduino через последовательный резистор 1К.
    При использовании для питания модуля собственного источника 3,3В Arduino, необходимо помнить о том,
    что модули могут потреблять в режиме передачи токи до 200 мА. Поэтому необходимо уточнять
    нагрузочные характеристики применяемой Вами платы Arduino. При коротких эфирных пакетах для
    компенсации недостаточного выходного тока источника 3,3В можно применить конденсаторы с емкостью
    не менее 2200 мкФ, устанавливаемые параллельно питанию модуля.
    На обоих модулях, после загрузки программного обеспечения SerialStar, должен быть произведен возврат
    к заводским настройкам одним из двух способов:
    1. Быстрое 4-х кратное нажатие "SYSTEM BUTTON" (замыкание вывода №36 модуля на землю).
    2. С помощью командного режима: +++, AT RE<CR>, AT CN<CR>.

    Первый модуль должен быть предварительно настроен для работы в пакетном режиме с escape-
    символами AP=2. Режим аппаратного управления потоком (CTS/RTS) должен быть отключен.
    Последовательность для настройки: +++, AT AP2<CR>, AT CN<CR>.
    Для контроля процесса передачи, к Arduino должны быть подключены через резисторы 1К два светодиода.

    Второй модуль устанавливается на плату MB-USBridge, или любой другой UART-USB/UART-COM
    преобразователь с выходными уровнями 3,3 В, для подключения к компьютеру. На компьютере
    должна быть запущена любая терминальная программа с настройками порта 9600 8N1.
    Никакие дополнительные предварительные настройки второго модуля не требуются.
*/


#define PC_Serial nss
#define MBee_Serial Serial

uint8_t ssRX = 8;
uint8_t ssTX = 9;
SoftwareSerial nss(ssRX, ssTX);
SerialStar mbee = SerialStar();

uint16_t remoteAddress = 0x0001; //Адрес модема, которому будут передаваться данные.

char testString[] = "Hello world!";
uint8_t testArray [] = {200, 100};

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

void setup()
{
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(Green_Led, OUTPUT);
  pinMode(Red_Led, OUTPUT);
  pinMode(ForwardPin , INPUT);
  pinMode(BackwardPin, INPUT);
  PC_Serial.begin(9600);
  MBee_Serial.begin(9600);
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
  if (CommandForRaper != 0)
  {
    testArray[1] = CommandForRaper;
  }
  sendData(); // sending
  //  delay(300);
  // +1 counter of packet
  number_of_packet++;
  testArray [0] = number_of_packet;
}

void ReadCommandFromSwither()
{
  //Check  a move command from  switch(remote controller)
  bool Forward = digitalRead(ForwardPin);
  bool Backward = digitalRead(BackwardPin);
  // Forward command
  if (Forward == true & Backward == false)
  {
    CommandForRaper = 1;
    digitalWrite(Red_Led,  LOW);
    digitalWrite(Green_Led, HIGH);
    PC_Serial.println("Command RUSH ");
  }
  // Backward command
  if (Forward == false & Backward == true)
  {
    CommandForRaper = 101;
    digitalWrite(Red_Led,  HIGH);
    digitalWrite(Green_Led, LOW);
    PC_Serial.println("Command BACK ");
  }
  // Stop command
  if (Forward == false & Backward == false)
  {
    CommandForRaper = 202;
    digitalWrite(Red_Led,  LOW);
    digitalWrite(Green_Led, LOW);
    PC_Serial.println("Command STOP !!! ");
  }
  // Error
  if (Forward == true & Backward == true)
  {
    PC_Serial.println();
    PC_Serial.println("I take Error from \"switcher\" , =( ");
    PC_Serial.println();
    CommandForRaper = 0;
    digitalWrite(Red_Led,  HIGH);
    digitalWrite(Green_Led, HIGH);
    digitalWrite(statusLed, HIGH);
    PC_Serial.println(" NO Command , we have a ERROR ((( ");
  }
  PC_Serial.println("CommandForRaper == ");
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
