/**
   Этот файл является частью библиотеки MBee-Arduino.
   MBee-Arduino является бесплатным программным обеспечением.
   Подробная информация о лицензиях находится в файле mbee.h.
   \author </i> von Boduen. Special thanx to Andrew Rapp.
*/

#include <MBee.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>

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

// ОПИСАНИЕ ЗАДАЧИ ДОБАВЬ!


//For UNO
//#define PC_Debug nss
#define PortToProcessing nss2
#define MBee_Serial Serial

//For MEGA
//#define PC_Debug Serial
//#define MBee_Serial Serial1

//Const
#define Stop 0
#define Forward 5
#define Backward 2
#define Corrupted_Command 10
#define ZeroPWM 30
#define Max_SpeedValue 225
#define Min_SpeedValue -225

//ADC read pins
#define ADCpin_Current1 0
#define ADCpin_Speed1 1

//PINs
#define ForwardLed 4
#define BackwardLed 5
#define PWM_Pin 6
// 8 + 9 -> RX/TX
//11 ???
#define BUZZER_PIN 10
#define LedLimited 12
#define systemLed 13   //#define errorLed 13 statusLed
int statusLed = LED_BUILTIN; //Используется встроенный в Вашу плату Arduino светодиод.

int SpeedValue_Now = ZeroPWM; // stop command
int Step_For_Move = 0;

SerialStar mbee = SerialStar();
RxResponse rx = RxResponse();
uint16_t remoteAddress = 0x0001; //Адрес модема, которому будут передаваться данные.
TxRequest tx = TxRequest(); //Пакет с данными для отправки удаленному модему. Для экономии памяти будем использовать один и тот же объект для отправки всех пакетов.
TxStatusResponse txStatus = TxStatusResponse(); //Локальный ответ со статусом команды.
RxAcknowledgeResponse remoteRxStatus = RxAcknowledgeResponse(); //Пакет с подтверждением от удаленного модема получение данных.

int Permission_Of_Move = 2; // orange wire - debug
int Direction_Of_Move = 3; // yellow wire - debug 111
//int statusLed = 12;
int InputValue = 0;
int CounterOfPacketsFromTx = 0;
uint8_t option = 0;
uint8_t data = 0;
uint8_t ssRX = 8;
uint8_t ssTX = 9;
int cntr = 0;
int E1 = 0; // Failed Checksum
int E2 = 0; // Wrong length of RX packet
int E3 = 0; // Not correct Start byte
int E4 = 0; // Corrupted frame
int ErrorSum = 0;
int ChainComboErrors = 0;
bool WDT_ACTIVE = false;
bool AlarmTrigger = false;
int GoodRx = 0;
double KoefGood = 0;
int Current_1 = 0;
int Speed_1 = 0;
int Current_2 = 0;
int Speed_2 = 0;
//SoftwareSerial nss(ssRX, ssTX);
SoftwareSerial nss2(ssRX, ssTX);

//Обработка прерывания по переполнению счётчика. Должны пищалкой пищать
ISR(TIMER2_OVF_vect) {
  cli();
  TCNT2 = 0; //55; Костыльное решение для изменения скорости моргания. ЛОЛ
  cntr++;
  if (cntr > 1000)
  {
    // ALARM
    Alarm_ON();
    if (WDT_ACTIVE == false)
    {
      wdt_enable(WDTO_4S); // WDT ENABLE!
      WDT_ACTIVE = true;
    }
    cntr = 0;
    digitalWrite(Direction_Of_Move  , LOW);
    digitalWrite(Permission_Of_Move, LOW);
    SpeedValue_Now = 0;
    analogWrite(PWM_Pin , SpeedValue_Now);
    // command stop.
  }
  sei();
}

void Reset_Error_Timer_And_Check_WDT()
{
  cli();
  if (AlarmTrigger == true)
  {
    Alarm_OFF();
    if (WDT_ACTIVE = true)
    {
      WDT_ACTIVE = false;
      wdt_disable();           //TURN OFF WDT!
    }
  }
  TCCR2B = 0b00000000;     // 0b00000000 -> STOP_TIMER (CS02 = CS01 = CS00 = 0)
  TCNT2 = 0;  // обнуляем регистр счёта
  TCCR2B = 0b00000011;     // 0b00000010 -> clk/8 (CS02 = 0 / CS01 = 1 / CS00 = 0)
  cntr = 0;
  sei();
}

void Alarm_ON()
{
  AlarmTrigger = true;
  digitalWrite(BUZZER_PIN, HIGH);
}

void Alarm_OFF()
{
  AlarmTrigger = false;
  digitalWrite(BUZZER_PIN, LOW);
}

void setup()
{
  pinMode(statusLed, OUTPUT);
  //  pinMode(errorLed, OUTPUT);
  pinMode(Permission_Of_Move, OUTPUT);
  pinMode(Direction_Of_Move, OUTPUT);
  pinMode(LedLimited, OUTPUT);
  //  PC_Debug.begin(115200); // 9600

  PortToProcessing.begin(74880);
  MBee_Serial.begin(115200);
  mbee.begin(MBee_Serial);

  //  PC_Debug.println("Start_Debug!");
  digitalWrite(LedLimited, LOW);

  //Настройка таймера 2 (T2)
  TCCR2A = 0;     // 0b00000000 -> Normal режим (никаких ШИМ) / Не ставили условий для ноги OC0A(12)
  TCNT2 = 0;      // 0b00111011 -> Это просто регистр со значениями куда всё тикает. зачем там 59?
  TIMSK2 = 0b00000001;  //0b00000001 - TOIE - до переполнения // ИЛИ -> TIMSK2 = 1; ИЛИ -> TIMSK2 |= (1 << TOIE2);
  TCCR2B = 0b00000011;  // 0b00000010 -> clk/8 (CS02 / CS01 / CS00)

  delay(500); //Задержка не обязательна и вставлена для удобства работы с терминальной программой.
}

void loop()
{
  //  PC_Debug.println("---------------");
  //Receive message from picocom
  mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
  if (mbee.getResponse().isAvailable())
  {
    //    PC_Debug.println("READED PACKET!!!");
    ChainComboErrors = 0; // reset combo
    if (mbee.getResponse().getApiId() == RECEIVE_PACKET_API_FRAME || mbee.getResponse().getApiId() == RECEIVE_PACKET_NO_OPTIONS_API_FRAME) // 0x81==129 and 0x8F==143
    {
      Reset_Error_Timer_And_Check_WDT();
      Alarm_OFF();
      mbee.getResponse().getRxResponse(rx); //Получаем пакет с данными.

      //      Debug info
      //      Debug_information_About_Rx_Packet();
      //      PC_Debug.println( mbee.getResponse().getApiId() ); // tut RECEIVE_PACKET_NO_OPTIONS_API_FRAME == 143 == 0x8F
      int8_t size = rx.getDataLength();

      //Debug
      //      bool WriteAllArrayFlag = false;
      //      Print_All_Array(size, WriteAllArrayFlag);
      //"WRITE" ALL PACKET
      //      if (rx.getDataLength() > 1)
      //      {
      //        PC_Debug.println();
      //        PC_Debug.print("Input Data :");
      //        for (int i = 0 ; i < rx.getDataLength() ; i++ )
      //        {
      //          PC_Debug.write(rx.getData()[i]);
      //        }
      //        PC_Debug.println();
      //      }
      CounterOfPacketsFromTx = rx.getData()[8]; // 8ой байт номер пакета посланный с хоста (маркер)
      //      PC_Debug.print("Counter Tx Packets :");
      //      PC_Debug.println(CounterOfPacketsFromTx);
      //      PC_Debug.println();
      InputValue = rx.getData()[9];  // 9ый байт наша команда (куда ехать)
      //      PC_Debug.print("Input Value:");
      //      PC_Debug.println(InputValue);
      if (rx.getData()[10] == 170)
      {
        InputValue = Stop;
        EmergencyStop();
        //        PC_Debug.println("EMERGENCY STOP");
        bool WriteAllArrayFlag = false;
        Print_All_Array(size, WriteAllArrayFlag);
        delay(100);
      }
      // Wacth on this Value
      //      if (InputValue > 0 & InputValue < 100 )
      //      {
      //        digitalWrite(Permission_To_Move, HIGH);
      //        digitalWrite(Diretction_To_Move, HIGH);
      //        PC_Debug.println("RUSHER");
      //
      //      }
      //Processing input value
      Command_To_Motor(InputValue);
      //fix for cycle errors
      // HOW???
      GoodRx = GoodRx + 1;
    }
    else
    {
      //      PC_Debug.println("Corrupted frame (maybe )");
      E4 = E4 + 1;
      ChainComboErrors = ChainComboErrors + 1;
      delay(100);
    }
    flashLed(statusLed, 1, 500); //Принят фрейм, не предназначенный для передачи неструктурированных данных.
  }
  else if (mbee.getResponse().isError())
  {
    //    PC_Debug.print("При разборе принятого пакета произошли ошибки.");
    //    PC_Debug.print(" № == ");
    ChainComboErrors = ChainComboErrors + 1;
    //    PC_Debug.println(mbee.getResponse().getErrorCode());
    switch (mbee.getResponse().getErrorCode())
    {
      case 1:
        E1 = E1 + 1;
        break;
      case 2:
        E2 = E2 + 1;
        break;
      case 3:
        E3 = E3 + 1;
        break;
    }
    //Debug Errored array
    int8_t size = rx.getDataLength();
    bool WriteAllArrayFlag = false;
    Print_All_Array(size, WriteAllArrayFlag);
    flashLed(statusLed, 2, 100); //При разборе принятого пакета произошли ошибки.
  }
  // Send Message
  //GOTO
  if (ChainComboErrors >= 100 )
  {
    //    PC_Debug.println("RESET ARDUINO. A terrible long combo ERRORS !!!"); //RESET
    WDT_ACTIVE = true;
    wdt_enable(WDTO_15MS); // WDT ENABLE!
    delay(50);
  }

  //  PC_Debug.println("---------------");
  //  PC_Debug.println("");
  //  delay(2000);
  ErrorSum = E1 + E2 + E3 + E4;
  if (ErrorSum >= 500)
  {
    Alarm_ON();
    //    PC_Debug.print("ErrorSum = ");
    //    PC_Debug.println(ErrorSum);
    //    PC_Debug.print("GoodRx = ");
    //    PC_Debug.println(GoodRx);
    //    PC_Debug.print("E1 = ");
    //    PC_Debug.println(E1);
    //    PC_Debug.print("E2 = ");
    //    PC_Debug.println(E2);
    //    PC_Debug.print("E3 = ");
    //    PC_Debug.println(E3);
    //    PC_Debug.print("E3 = ");
    //    PC_Debug.println(E3);
  }
  //    while(1);
  ADCread();
  Data_Send_To_Processing();
}

void Command_To_Motor(int instruction)
{
  if (instruction == Corrupted_Command)
  {
    // NEED ADD REACTION On "ERROR_VALUE"
    return;
  }
  //Stop
  if (instruction == Stop)
  {
    if (SpeedValue_Now == ZeroPWM)
    {
      //      PC_Debug.println("We stopped + command already complete");
      Step_For_Move = 0; // command already complete
    }
    if (SpeedValue_Now > ZeroPWM)
    {
      Step_For_Move = -4; // pull to zero // 1 - FOR TESTS , 7 - for real fights
    }
    if (SpeedValue_Now < ZeroPWM)
    {
      Step_For_Move = 4; // pull to zero // 1 - FOR TESTS , 7 - for real fights
    }
    //Calculate
    SpeedValue_Now = SpeedValue_Now + Step_For_Move;
    //Teleport
    if ( ( (SpeedValue_Now <= ZeroPWM) & (SpeedValue_Now >= ZeroPWM - abs(Step_For_Move)) )                       //  (X <= 30) & (X >= 25)
         | ( (SpeedValue_Now >= (-1) * ZeroPWM) & (SpeedValue_Now <= ( (-1) * ZeroPWM + abs(Step_For_Move)) ) ) ) // ( X >= -30) & ( X <= -25 )
    {
      SpeedValue_Now = ZeroPWM;
      Step_For_Move = 0;  // command complete
      //      PC_Debug.println("STOP NOW");
      digitalWrite(LedLimited, LOW);
    }
    //Debug
    //    PC_Debug.print("(Stopping)Speed == ");
    //    PC_Debug.println(SpeedValue_Now);
    //    PC_Debug.println("---------------");
  }
  //Forward
  if (instruction == Forward)
  {
    if (SpeedValue_Now == Max_SpeedValue) // check for Wished_Speed
    {
      //      PC_Debug.println("Max Speed is reached! (already)");
      Step_For_Move = 0;
    }
    if (SpeedValue_Now >= Min_SpeedValue && SpeedValue_Now < Max_SpeedValue)
    {
      Step_For_Move = 4; // pull to 225 (Max_SpeedValue)
    }
    SpeedValue_Now = SpeedValue_Now + Step_For_Move;
    //Teleport
    if (SpeedValue_Now >= ZeroPWM * (-1) & (SpeedValue_Now <= ((-1) * ZeroPWM + abs(Step_For_Move)) ) )  // (X >= -30 & X <= -25 )
      SpeedValue_Now = ZeroPWM;
    // FINISHER
    if (SpeedValue_Now >= Max_SpeedValue & SpeedValue_Now <= ( Max_SpeedValue + abs(Step_For_Move) )) // ==Wished_Speed
    {
      SpeedValue_Now = Max_SpeedValue;
      Step_For_Move = 0;  // command complete
      digitalWrite(LedLimited, HIGH);
      //      PC_Debug.println("Max Speed RIGHT NOW! LvL UP! ");
    }
    //Debug
    //    PC_Debug.print("(Grow) Speed == ");
    //    PC_Debug.println(SpeedValue_Now);
    //    PC_Debug.println("---------------");
  }
  //Backward
  if (instruction == Backward)
  {
    if (SpeedValue_Now == Min_SpeedValue)
    {
      //      PC_Debug.println("Min Speed is reached! (already)");
      Step_For_Move = 0; // pull to 255 (Max_SpeedValue)
    }
    if (SpeedValue_Now > Min_SpeedValue && SpeedValue_Now <= Max_SpeedValue)
    {
      Step_For_Move = -4;  // -1 - FOR TESTS , -7 - for real fights
    }
    //Teleport
    SpeedValue_Now = SpeedValue_Now + Step_For_Move;
    if ( (SpeedValue_Now <= ZeroPWM) & (SpeedValue_Now >= ZeroPWM - abs(Step_For_Move) ) ) // == 30 |  ( X <= 30 & X >= 25)
      SpeedValue_Now = ZeroPWM * (-1);
    // FINISHER
    if (SpeedValue_Now <= Min_SpeedValue)
    {
      SpeedValue_Now = Min_SpeedValue;
      Step_For_Move = 0;  // command complete
      digitalWrite(LedLimited, HIGH);
      //      PC_Debug.println("We Reversed");
    }
    //Debug
    //    PC_Debug.print("(Drop) Speed == ");
    //    PC_Debug.println(SpeedValue_Now);
  }
  // Command Release
  Execute_The_Command(SpeedValue_Now);
  //  delay(10);
  return;
}

void Execute_The_Command(int Speed)
{
  if (Speed >= Min_SpeedValue & Speed <= Max_SpeedValue)
  {
    if (Speed > ZeroPWM) //F
    {
      digitalWrite(Direction_Of_Move, HIGH);
      digitalWrite(Permission_Of_Move, HIGH);
      analogWrite(PWM_Pin, Speed);
      //Debug
      //      PC_Debug.println("FORWARD --->>>>");
      digitalWrite(ForwardLed, HIGH);
      digitalWrite(BackwardLed, LOW);
      digitalWrite(systemLed, LOW);
    }
    if (Speed < ZeroPWM) //B
    {
      digitalWrite(Direction_Of_Move, LOW);
      digitalWrite(Permission_Of_Move, HIGH);
      analogWrite(PWM_Pin, abs(Speed)); // since it is less than 0
      //Debug
      //      PC_Debug.println("<<<<--- BACKWARD");
      digitalWrite(ForwardLed, LOW);
      digitalWrite(BackwardLed, HIGH);
      digitalWrite(systemLed, LOW);
    }
    if (Speed == ZeroPWM) //S
    {
      digitalWrite(Direction_Of_Move, LOW);
      digitalWrite(Permission_Of_Move, LOW);
      analogWrite(PWM_Pin, Speed);
      //Debug
      //      PC_Debug.println("|| STOP ||");
      digitalWrite(ForwardLed, LOW);
      digitalWrite(BackwardLed, LOW);
      digitalWrite(systemLed, LOW);
    }
  }
  else
  {
    // STOP RIGHT NOW
    digitalWrite(Direction_Of_Move, LOW);
    digitalWrite(Permission_Of_Move, LOW);
    analogWrite(PWM_Pin, ZeroPWM);
    SpeedValue_Now = ZeroPWM;
    Step_For_Move = 0;
    //Debug
    //    PC_Debug.print("Error!!! Speed == ");
    //    PC_Debug.println(SpeedValue_Now);
    //    PC_Debug.println("");
    //    PC_Debug.print("RESET -> Speed Value_Now = ");
    //    PC_Debug.println(SpeedValue_Now);
    //    PC_Debug.println("");
    //      PC_Debug.println("ERROR from Host (Swither have bugs or lags) , sadness ");
    //Debug on LEDs
    digitalWrite(ForwardLed, HIGH);
    digitalWrite(BackwardLed, HIGH);
    digitalWrite(systemLed, HIGH);
  }
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
  //  PC_Debug.println("TX NOW ");
  mbee.send(tx); // по сути сама отправка
  //  PC_Debug.print("tx.getFrameId() == ");
  //  PC_Debug.println( tx.getFrameId() );
  // TEST
  if (tx.getFrameId()) //Проверяем, не заблокировано ли локальное подтверждение отправки.
    getLocalResponse(50);
  if ((tx.getFrameId() == 0) || (txStatus.isSuccess() && tx.getSleepingDevice() == false)) //Ждем ответного пакета от удаленного модема
    //только если локальный ответ выключен или пакет отправлен в эфир и не предназначается спящему модему.
    getRemoteResponse(100);
}

void getLocalResponse(uint16_t timeout)
{
  if (mbee.readPacket(timeout)) //Ждем ответа со статусом команды.
  {
    //    PC_Debug.print("getLocalResponse -> mbee.getResponse().getApiId() ==");
    //    PC_Debug.println( mbee.getResponse().getApiId() );
    if (mbee.getResponse().getApiId() == TRANSMIT_STATUS_API_FRAME) //Проверяем, является ли принятый пакет локальным ответом на AT-команду удаленному модему.
    {
      mbee.getResponse().getTxStatusResponse(txStatus);
      if (txStatus.isSuccess())
      {
        //        PC_Debug.println("Пакет ушел в эфир");
        flashLed(statusLed, 1, 200); //Порядок. Пакет ушел в эфир.
        //        delay(5000);
      }
      else if (txStatus.getStatus() == TX_FAILURE_COMMAND_STATUS) // TX_FAILURE_COMMAND_STATUS == 4
      {
        //        PC_Debug.println("Пакет в эфир не ушел вследствие занятости канала");
        flashLed(statusLed, 1, 200); //Пакет в эфир не ушел вследствие занятости канала.
      }
      else if (txStatus.getStatus() == ERROR_COMMAND_STATUS) // ERROR_COMMAND_STATUS == 1
      {
        //        PC_Debug.println("Недостаточно памяти для размещения пакета в буфере на передачу");
        flashLed(statusLed, 1, 1000); //Недостаточно памяти для размещения пакета в буфере на передачу.
      }
    }
  }
}

void getRemoteResponse(uint16_t timeout)
{
  if (mbee.readPacket(timeout)) //Ждем подтверждения получения данных от удаленного модуля.
  {
    //    PC_Debug.print("getRemoteResponse -> mbee.getResponse().getApiId() == ");
    //    PC_Debug.println(mbee.getResponse().getApiId());
    if (mbee.getResponse().getApiId() == REMOTE_ACKNOWLEDGE_API_FRAME) //Является ли полученный фрейм подтверждением доставки.
    {
      mbee.getResponse().getRxAcknowledgeResponse(remoteRxStatus);
      if (remoteRxStatus.getFrameId() == tx.getFrameId()) //Проверяем,совпадает ли идентификатор фрейма в полученном подтверждении с идентификатором отправленного пакета.
        flashLed(statusLed, 2, 200); //Подтверждение получено именно на отпправленный только что пакет. Все прекрасно.
      else
        flashLed(statusLed, 2, 200); //Идентификатор пакета в принятом подтверждении не совпал с идентификтором отправленного пакета.
    }
    else
      flashLed(statusLed, 3, 200); //Принятый пакет не является пакетом подтверждения доставки.
  }
  else
  {
    if (mbee.getResponse().isError())
      flashLed(statusLed, 4, 200); //В процессе разбора принятого пакета произошли ошибки.
    else
      flashLed(statusLed, 1, 2000); //Нет ответа от удаленного модема.
  }
}

void Debug_information_About_Rx_Packet()
{
  //  PC_Debug.println("---------------------DEBUG------------------");
  //  //Print of Received Packet  Length
  //  PC_Debug.print("Packet Length -> ");
  //  PC_Debug.println(rx.getPacketLength());
  //  //Print of MSB Length
  //  PC_Debug.print("MSB Lenght -> ");
  //  PC_Debug.println(rx.getMsbLength());
  //  //Print of LSB Length
  //  PC_Debug.print("LSB Lenght -> ");
  //  PC_Debug.println(rx.getLsbLength());
  //  //Print of API ID
  //  PC_Debug.print("API ID -> ");
  //  PC_Debug.println(rx.getApiId());
  //  // Print Checksum
  //  PC_Debug.print("Checksum == ");
  //  PC_Debug.println(rx.getChecksum());
  //  // Print Frame Data Length
  //  PC_Debug.print("Frame Data Lenght -> ");
  //  PC_Debug.println(rx.getFrameDataLength());
  //  // Print Data Length
  //  PC_Debug.print("Data Length -> ");
  //  PC_Debug.println(rx.getDataLength());
  //  // Print RSSI
  //  PC_Debug.print("RSSI -> ");
  //  PC_Debug.println(rx.getRssi());
  //
  //  PC_Debug.println("---------------------DEBUG------------------");
}

void Print_All_Array(int8_t Size, boolean WriteEnable)
{
  // ALL PACK PRINT!!
  //  PC_Debug.print("Print of Data [0] -> ");
  //  PC_Debug.println(Size);
  for (int i = 0 ; i < Size ; i ++ )
  {
    //PRINT
    //    PC_Debug.print("Input testArray[");
    //    PC_Debug.print(i);
    //    PC_Debug.print("]= ");
    //    PC_Debug.println(rx.getData()[i]);
    //WRITE
    if (WriteEnable == true)
    {
      //      PC_Debug.print(" WRITE inside i -> ");
      //      PC_Debug.write(rx.getData()[i]);
      //      PC_Debug.println();
    }
  }
}

void EmergencyStop()
{
  Alarm_ON();
  // STOP RIGHT NOW
  SpeedValue_Now = ZeroPWM;
  Step_For_Move = 0;
  digitalWrite(Direction_Of_Move, LOW);
  digitalWrite(Permission_Of_Move, LOW);
  digitalWrite(Permission_Of_Move, LOW);
  analogWrite(PWM_Pin, ZeroPWM);
}


void ADCread() {
  Current_1 = analogRead(ADCpin_Current1);
  Speed_1 = analogRead(ADCpin_Speed1);
  return;
}

void Data_Send_To_Processing() {
  int Garbage = random(-1023,1023);
  PortToProcessing.print("Speed_1=");
  PortToProcessing.print(Speed_1);
  PortToProcessing.print(" ");

  PortToProcessing.print("Speed_2=");
  PortToProcessing.print(Speed_2);
  PortToProcessing.print(" ");

  PortToProcessing.print("I_1=");
  PortToProcessing.print(Current_1);
  PortToProcessing.print(" ");

  PortToProcessing.print("I_2=");
  PortToProcessing.print(Current_2);
  PortToProcessing.print(" ");

  //  Serial.println(val);
  PortToProcessing.print("Garbage=");
  PortToProcessing.print(Garbage);
  PortToProcessing.print(" ");
  PortToProcessing.println();
  delay(30);
}
