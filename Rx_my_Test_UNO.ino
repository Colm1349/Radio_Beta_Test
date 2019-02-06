/**
   Этот файл является частью библиотеки MBee-Arduino.

   MBee-Arduino является бесплатным программным обеспечением.
   Подробная информация о лицензиях находится в файле mbee.h.

   \author </i> von Boduen. Special thanx to Andrew Rapp.
*/

#include <MBee.h>
#include <SoftwareSerial.h>

/**
    Скетч предназначен для демонстрации приема пакетов с неструктурированными данными от удаленного модема.
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
    Для контроля приема данных, к Arduino должны быть подключены через резисторы 1К два светодиода.

    Второй модуль устанавливается на плату MB-USBridge, или любой другой UART-USB/UART-COM
    преобразователь с выходными уровнями 3,3 В, для подключения к компьютеру. На компьютере
    должна быть запущена любая терминальная программа с настройками порта 9600 8N1.
    С ее помощью осуществляется передача кодов, соответствующих цифровым клавишам.
    Данный скетч предназначет только для приема пакетов с длиной поля данных 1 байт.
    Диапазон допустимых значений 0x31..0x39, что соответствует кнопкам 1..9.
    При успешном приеме, светодиод statusLed будет мигать в соответствии с нажатой цифровой клавишей.
    Никакие дополнительные предварительные настройки второго модуля не требуются.
*/

#define PC_Serial nss

SerialStar mbee = SerialStar();
RxResponse rx = RxResponse();
uint16_t remoteAddress = 0x0001; //Адрес модема, которому будут передаваться данные.
char testString[] = " Hell World!";
TxRequest tx = TxRequest(); //Пакет с данными для отправки удаленному модему. Для экономии памяти будем использовать один и тот же объект для отправки всех пакетов.
TxStatusResponse txStatus = TxStatusResponse(); //Локальный ответ со статусом команды.
RxAcknowledgeResponse remoteRxStatus = RxAcknowledgeResponse(); //Пакет с подтверждением от удаленного модема получение данных.
int errorLed = 11;
int statusLed = LED_BUILTIN; //Используется встроенный в Вашу плату Arduino светодиод.
int Permission_To_Move = 2; // orange wire - debug
int Diretction_To_Move = 3; // yellow wire - debug 111
//int statusLed = 12;
int InputValue = 0;
uint8_t option = 0;
uint8_t data = 0;
uint8_t ssRX = 8;
uint8_t ssTX = 9;
SoftwareSerial nss(ssRX, ssTX);


void setup()
{
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(Permission_To_Move, OUTPUT);
  pinMode(Diretction_To_Move, OUTPUT);
  PC_Serial.begin(9600);
  Serial.begin(9600);
  mbee.begin(Serial);

  PC_Serial.println("Start_Debug!");
  delay(500); //Задержка не обязательна и вставлена для удобства работы с терминальной программой.
}

void loop()
{
  //Receive message from picocom
  mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
  if (mbee.getResponse().isAvailable())
  {
    PC_Serial.println("READED PACKET!!!");
    // Print Data
    //    PC_Serial.print("Data [0] -> ");
    //    PC_Serial.println(rx.getDate()[0]);
    if (mbee.getResponse().getApiId() == RECEIVE_PACKET_API_FRAME || mbee.getResponse().getApiId() == RECEIVE_PACKET_NO_OPTIONS_API_FRAME) // 0x81==129 and 0x8F==143
    {
      mbee.getResponse().getRxResponse(rx); //Получаем пакет с данными.
      //      Debug info
      //      Debug_information_About_Rx_Packet();
      //Ниже приводятся некоторые функции доступа к различным полям пакета.
      //uint16_t remoteAddress = rx.getRemoteAddress(); //Получение адреса отправителя.
      //uint8_t rssi = rx.getRssi(); //Получения уровня сигнала на входе модуля на момент приема данного пакета. Передается в виде числа со знаком в дополнительном коде.
      //bool broadcastIndicator = rx.isAddressBroadcast(); //Является ли принятый пакет широковещательным?
      //bool acknowledgeIndicator = rx.isAcknowledged(); //Было ли отправлено подтверждение приема данного пакета?
//      PC_Serial.print("mbee.getResponse().getApiId() == ");
      PC_Serial.println( mbee.getResponse().getApiId() ); // tut RECEIVE_PACKET_NO_OPTIONS_API_FRAME == 143 == 0x8F
      int8_t size = rx.getDataLength();
      bool WriteAllArrayFlag = false;
//      Print_All_Array(size, WriteAllArrayFlag);
      
      //"WRITE" ALL PACKET
//      if (rx.getDataLength() > 1)
//      {
//        PC_Serial.println();
//        PC_Serial.print("Input Data :");
//        for (int i = 0 ; i < rx.getDataLength() ; i++ )
//        {
//          PC_Serial.write(rx.getData()[i]);
//        }
//        PC_Serial.println();

//      }

      InputValue = rx.getData()[9];
      PC_Serial.print("Input Value:");
      PC_Serial.println(InputValue);
      PC_Serial.println();
      // Wacth on this Value
      if (InputValue > 0 & InputValue < 100 )
      {
        digitalWrite(Permission_To_Move, HIGH);
        digitalWrite(Diretction_To_Move, HIGH);
        PC_Serial.println("RUSHER");

      }
      if (InputValue >= 100 & InputValue < 150 )
      {
        digitalWrite(Permission_To_Move, HIGH);
        digitalWrite(Diretction_To_Move, LOW);
        PC_Serial.println("BACK");
      }
      if ( InputValue >= 150 & InputValue < 255)
      {
        digitalWrite(Permission_To_Move,  LOW);
        digitalWrite(Diretction_To_Move, LOW);
        PC_Serial.println("STOP");
      }
      if (InputValue == 0)
      {
        digitalWrite(Permission_To_Move,  LOW);
        digitalWrite(Diretction_To_Move, LOW);
        PC_Serial.println("ERROR from Host (Swither have bugs or lags) , sadness ");
      }
    }
    else
    {
      delay(1);
      PC_Serial.println("Corrupted frame (maybe )");
    }
    //      flashLed(errorLed, 1, 500); //Принят фрейм, не предназначенный для передачи неструктурированных данных.
  }
  else if (mbee.getResponse().isError())
  {
    PC_Serial.println("При разборе принятого пакета произошли ошибки.");
    //    flashLed(errorLed, 2, 100); //При разборе принятого пакета произошли ошибки.
  }
  // Send Message
  //  tx.setRemoteAddress(remoteAddress);  //Устанавливаем адрес удаленного модема. // remoteAddress == 0x0001
  //  tx.setPayload((uint8_t*)testString); //Устанавливаем указатель на тестовую строку.
  //  tx.setPayloadLength(sizeof(testString) - 1); //Устанавливаем длину поля данных на 1 меньше, чем получаем по операции sizeof, чтобы не передавать терминирующий символ 0x00.
  //  sendData();

  //  while(1);
}

void flashLed(int pin, int times, int wait)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(pin, HIGH);
    //    delay(wait);
    delay(1);
    digitalWrite(pin, LOW);
    if (i + 1 < times)
      //      delay(wait);
      delay(1);
  }
}

void sendData()
{
  PC_Serial.println("TX NOW ");
  mbee.send(tx); // по сути сама отправка
  PC_Serial.print("tx.getFrameId() == ");
  PC_Serial.println( tx.getFrameId() );
  if (tx.getFrameId()) //Проверяем, не заблокировано ли локальное подтверждение отправки.
    getLocalResponse(50);
  if ((tx.getFrameId() == 0) || (txStatus.isSuccess() && tx.getSleepingDevice() == false)) //Ждем ответного пакета от удаленного модема только если локальный ответ выключен или пакет отправлен в эфир и не предназначается спящему модему.
    getRemoteResponse(100);
}

void getLocalResponse(uint16_t timeout)
{
  if (mbee.readPacket(timeout)) //Ждем ответа со статусом команды.
  {
    PC_Serial.print("getLocalResponse -> mbee.getResponse().getApiId() ==");
    PC_Serial.println( mbee.getResponse().getApiId() );
    if (mbee.getResponse().getApiId() == TRANSMIT_STATUS_API_FRAME) //Проверяем, является ли принятый пакет локальным ответом на AT-команду удаленному модему.
    {
      mbee.getResponse().getTxStatusResponse(txStatus);
      if (txStatus.isSuccess())
      {
        PC_Serial.println("Пакет ушел в эфир");
        flashLed(statusLed, 1, 200); //Порядок. Пакет ушел в эфир.
        //        delay(5000);
      }
      else if (txStatus.getStatus() == TX_FAILURE_COMMAND_STATUS) // TX_FAILURE_COMMAND_STATUS == 4
      {
        PC_Serial.println("Пакет в эфир не ушел вследствие занятости канала");
        flashLed(errorLed, 1, 200); //Пакет в эфир не ушел вследствие занятости канала.
      }
      else if (txStatus.getStatus() == ERROR_COMMAND_STATUS) // ERROR_COMMAND_STATUS == 1
      {
        PC_Serial.println("Недостаточно памяти для размещения пакета в буфере на передачу");
        flashLed(errorLed, 1, 1000); //Недостаточно памяти для размещения пакета в буфере на передачу.
      }
    }
  }
}

void getRemoteResponse(uint16_t timeout)
{
  if (mbee.readPacket(timeout)) //Ждем подтверждения получения данных от удаленного модуля.
  {
    PC_Serial.print("getRemoteResponse -> mbee.getResponse().getApiId() == ");
    PC_Serial.println(mbee.getResponse().getApiId());
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

void Debug_information_About_Rx_Packet()
{
  PC_Serial.println("---------------------DEBUG------------------");
  //Print of Received Packet  Length
  PC_Serial.print("Packet Length -> ");
  PC_Serial.println(rx.getPacketLength());
  //Print of MSB Length
  PC_Serial.print("MSB Lenght -> ");
  PC_Serial.println(rx.getMsbLength());
  //Print of LSB Length
  PC_Serial.print("LSB Lenght -> ");
  PC_Serial.println(rx.getLsbLength());
  //Print of API ID
  PC_Serial.print("API ID -> ");
  PC_Serial.println(rx.getApiId());
  // Print Checksum
  PC_Serial.print("Checksum == ");
  PC_Serial.println(rx.getChecksum());
  // Print Frame Data Length
  PC_Serial.print("Frame Data Lenght -> ");
  PC_Serial.println(rx.getFrameDataLength());
  // Print Data Length
  PC_Serial.print("Data Length -> ");
  PC_Serial.println(rx.getDataLength());
  // Print RSSI
  PC_Serial.print("RSSI -> ");
  PC_Serial.println(rx.getRssi());

  PC_Serial.println("---------------------DEBUG------------------");
}

void Print_All_Array(int8_t Size, boolean WriteEnable)
{
  // ALL PACK PRINT!!
  PC_Serial.print("Print of Data [0] -> ");
  PC_Serial.println(Size);
  for (int i = 0 ; i < Size ; i ++ )
  {
    //PRINT
    PC_Serial.print("Input testArray[");
    PC_Serial.print(i);
    PC_Serial.print("]= ");
    PC_Serial.println(rx.getData()[i]);
    //WRITE
    if (WriteEnable == true)
    {
      PC_Serial.print(" WRITE inside i -> ");
      PC_Serial.write(rx.getData()[i]);
      PC_Serial.println();
    }
  }
}

void serialEvent1()
{
//  PC_Serial.println("serialEvent1 for MBEE_ PROCK ");
}
