/**
   Этот файл является частью библиотеки MBee-Arduino.
   MBee-Arduino является бесплатным программным обеспечением.
   Подробная информация о лицензиях находится в файле mbee.h.
   \author </i> von Boduen. Special thanx to Andrew Rapp.
*/

#include <MBee.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>

// ОПИСАНИЕ ЗАДАЧИ ДОБАВЬ!


//For UNO
#define PC_Debug nss2
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
#define BUZZER_PIN 10
#define LedDebug 12
#define LedLimited 11
#define systemLed 13
int statusLed = LED_BUILTIN;

int SpeedValue_Now = ZeroPWM; //stop command
int Step_For_Move = 0;
//CounterRxPackets (0),Speed_1H/L (1-2) , Speed_2H/L (3-4), I_1H/L (5-6), I_2H/L (7-8), V_Roper_H/L (9-10), PWM_Value (11), Direction (12)
uint8_t testArray [] = {0, 111, 22, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
SerialStar mbee = SerialStar();
RxResponse rx = RxResponse();
uint16_t remoteAddress = 0x0001; //Адрес модема, которому будут передаваться данные.
TxRequest tx = TxRequest(); //Пакет с данными для отправки удаленному модему. Для экономии памяти будем использовать один и тот же объект для отправки всех пакетов.
TxStatusResponse txStatus = TxStatusResponse(); //Локальный ответ со статусом команды.
RxAcknowledgeResponse remoteRxStatus = RxAcknowledgeResponse(); //Пакет с подтверждением от удаленного модема получение данных.

int Permission_Of_Move = 2; // orange wire - debug
int Direction_Of_Move = 3; // yellow wire - debug 111
int InputValue = 0;
int CounterOfPacketsFromTx = 0;
uint8_t option = 0;
uint8_t data = 0;
uint8_t ssRX = 8;
uint8_t ssTX = 9;
int cntr = 0;
int Counter_To_Start_WDT = 0;
int E1 = 0; // Failed Checksum
int E2 = 0; // Wrong length of RX packet
int E3 = 0; // Not correct Start byte
int E4 = 0; // Corrupted frame
int ErrorSum = 0;
int ChainComboErrors = 0;
bool WDT_ACTIVE = false;
bool AlarmTrigger = false;
bool FLAG_Release_Command = false;
int GoodRx = 0;
double KoefGood = 0;
int Current_1 = 0;
int Speed_1 = 0;
int Current_2 = 0;
int Speed_2 = 0;
//SoftwareSerial nss(ssRX, ssTX);
SoftwareSerial nss2(ssRX, ssTX);

//Обработка прерывания по переполнению счётчика. Должны пищалкой пищать
ISR(TIMER2_OVF_vect)
{
  cli();
  TCCR2B = 0b00000000; // STOP COUNT!!
  // it means no signal from RC
  Counter_To_Start_WDT++; // overflow  x1000 then start WDT.
  if (Counter_To_Start_WDT > 1000)
  {
    //ALARM
    Alarm_ON();
    if (WDT_ACTIVE == false)
    {
      wdt_enable(WDTO_4S); // WDT ENABLE!
      WDT_ACTIVE = true;
    }
    Counter_To_Start_WDT = 0;
    //Stop NOW
    digitalWrite(Direction_Of_Move , LOW);
    digitalWrite(Permission_Of_Move, LOW);
    SpeedValue_Now = ZeroPWM;
    analogWrite(PWM_Pin , SpeedValue_Now);
  }
  //it must rotate non stop
  if (FLAG_Release_Command == false)
  {
    if (cntr > 300)
    {
      FLAG_Release_Command = true;
      cntr = 0;
    }
    else cntr++;
  }
  TCCR2B = 0b00000011; // START COUNT!!
  sei();
}

//ISR(TIMER2_COMPA_vect)
//{
//  //need text
//  cli();
//  int x = 240;
//  TCCR2B = 0b00000000;
//  static int counter_t0 = 0;
//  int val = 0;
//  // digitalWrite(LedDebug, !digitalRead(LedDebug));
//  // PC_Debug.print("____________T2_COMPA TCNT0 == ");
//  // PC_Debug.println(TCNT2);
//  // delay(500);
//  if (counter_t0 > 5)
//  {
//    counter_t0 = 0;
//    PC_Debug.println();
//    PC_Debug.print("TCNT2-> ");
//    PC_Debug.println(TCNT2);
//    Command_To_Motor(InputValue);
//    counter_t0 = 0;
//  }
//  counter_t0++;
//  //Command_To_Motor(InputValue);
//  //Command Release
//  //  Execute_The_Command(SpeedValue_Now);
//  TCCR2B = 0b00000011;
//  sei();
//}

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
  Counter_To_Start_WDT = 0;
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
  pinMode(LedDebug, OUTPUT);
  PC_Debug.begin(115200);
  MBee_Serial.begin(115200);
  mbee.begin(MBee_Serial);
  //  //Настройка таймера 0 (T1) 16 bit
  //  TCNT0 = 0;   // счетный регистр (туда "капают" значения)
  //  OCR0A =  200;           //0b1000000000000000 // регистр сравнения. (уставка)
  //  OCR0B =  0;
  //  TIMSK0 = 0b00000011; //0b00000010 - OCIE0A (1) разрешают прерывания при совпадении с регистром A (OCR1A)
  //  TIFR0  = 0;
  //  TCCR0A = 0b00000011; //0b00000000 -> Normal режим (никаких ШИМ) / Не ставили условий для ноги OC1A(12)
  //  TCCR0B = 0b00000011;  // WGM12 == 1 -> [0100] - режим счета импульсов (OCR1A) (сброс при совпадении) + 101 - CLK/1024 == 0b00001101;

  //Настройка таймера 2 (T2)
  TCNT2 = 0;       //  Это просто регистр со значениями куда всё тикает
  OCR2A =  100;    //уставка в регистр
  TIMSK2 = 0b00000001;  //0b00000001 - TOIE - до переполнения  + 0b00000010 до уставки в OCR2A // ИЛИ -> TIMSK2 = 1; ИЛИ -> TIMSK2 |= (1 << TOIE2);
  TCCR2A = 0;      // 0b00000000 -> Normal режим (никаких ШИМ) / Не ставили условий для ноги OC0A(12)
  TCCR2B = 0b00000011;  //START_TIMER -> 0b10 -> clk/8 (CS02 / CS01 / CS00) START_TIMER

  PC_Debug.println();
  PC_Debug.println("___RX started!___");
  Send_Telemetry(); //starting scream
  delay(500); //Задержка не обязательна и вставлена для удобства работы с терминальной программой.
}

void loop()
{
  ADCread();
  setArrayForTelemetry();
  tx.setPayload((uint8_t*)testArray); //Устанавливаем указатель на тестовый массив
  tx.setPayloadLength(sizeof(testArray));  //Устанавливаем длину поля данных
  mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
  if (mbee.getResponse().isAvailable())
  {
    PC_Debug.println("RX received an input packet!");
    ChainComboErrors = 0; // reset combo
    if (mbee.getResponse().getApiId() == RECEIVE_PACKET_API_FRAME || mbee.getResponse().getApiId() == RECEIVE_PACKET_NO_OPTIONS_API_FRAME) // 0x81==129 and 0x8F==143
    {
      Reset_Error_Timer_And_Check_WDT();
      Alarm_OFF();
      cli();
      mbee.getResponse().getRxResponse(rx); //Получаем пакет с данными.
      //      Debug info
      //      Debug_information_About_Rx_Packet();
      //      PC_Debug.println( mbee.getResponse().getApiId() ); // tut RECEIVE_PACKET_NO_OPTIONS_API_FRAME == 143 == 0x8F
      int8_t size = rx.getDataLength();
      //Debug
      //      bool WriteAllArrayFlag = false;
      //      Print_All_Array(size, WriteAllArrayFlag);
      if (rx.getDataLength() > 1)
      {
        PC_Debug.println("Input Data :");
        for (int i = 0 ; i < rx.getDataLength() ; i++ )
        {
          if ( i == 8 || i == 9) {
            PC_Debug.print("Value [");
            PC_Debug.print(i);
            PC_Debug.print("] = ");
            PC_Debug.println(rx.getData()[i]);
            if (i == 8) {
              CounterOfPacketsFromTx = rx.getData()[i]; // 8ой байт номер пакета посланный с хоста (маркер)
            }
          }
        }
      }
      PC_Debug.print("Counter Received Packets :");
      PC_Debug.println(CounterOfPacketsFromTx);
      PC_Debug.println();
      InputValue = rx.getData()[9];  // 9ый байт наша команда (куда ехать)
      if (rx.getData()[10] == 170)
      {
        InputValue = Stop;
        EmergencyStop();
        PC_Debug.println("EMERGENCY STOP");
        bool WriteAllArrayFlag = false;
        Print_All_Array(size, WriteAllArrayFlag);
        delay(100);
      }

      //Command_To_Motor(InputValue); // old version to move ours legs
      GoodRx = GoodRx + 1;
      sei();
    }
    else
    {
      PC_Debug.println("Corrupted frame (maybe )");
      E4 = E4 + 1;
      ChainComboErrors = ChainComboErrors + 1;
      delay(100);
    }
    /////////----------------------------------------------/////////
    Send_Telemetry();
    /////////----------------------------------------------/////////
  }
  else if (mbee.getResponse().isError())
  {
    PC_Debug.print("При разборе принятого пакета произошли ошибки.");
    PC_Debug.print(" № == ");
    PC_Debug.println(mbee.getResponse().getErrorCode());
    ChainComboErrors = ChainComboErrors + 1;
    delay(100);
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
  if (ChainComboErrors >= 100 )
  {
    PC_Debug.println("RESET ARDUINO. A terrible long combo ERRORS !!!"); //RESET
    WDT_ACTIVE = true;
    wdt_enable(WDTO_15MS); // WDT ENABLE!
    delay(50);
  }
  ErrorSum = E1 + E2 + E3 + E4;
  if (ErrorSum >= 500)
  {
    Alarm_ON();
    PC_Debug.print("ErrorSum = ");
    PC_Debug.println(ErrorSum);
    PC_Debug.print("GoodRx = ");
    PC_Debug.println(GoodRx);
    PC_Debug.print("E1 = ");
    PC_Debug.println(E1);
    PC_Debug.print("E2 = ");
    PC_Debug.println(E2);
    PC_Debug.print("E3 = ");
    PC_Debug.println(E3);
    PC_Debug.print("E4 = ");
    PC_Debug.println(E4);
  }
  /////////////////////////////////////////////////////////////////
  //  Data_Send_To_Processing();
  if (FLAG_Release_Command == true)
  {
    PC_Debug.println("Rotate again -_- SANSARA DRIFT");
    Command_To_Motor(InputValue);
    FLAG_Release_Command = false;
  }
  /////////////////////////////////////////////////////////////////
}

void Send_Telemetry()
{
  mbee.send(tx);
  PC_Debug.print("Current_1 ==");
  PC_Debug.println(Current_1);
  PC_Debug.print("Speed_1 ==");
  PC_Debug.println(Speed_1);
  PC_Debug.println("RX SEND telemetry.");
  //  if (tx.getFrameId()) //Проверяем, не заблокировано ли локальное подтверждение отправки.
  //    getLocalResponse(50);
  //  if ((tx.getFrameId() == 0) || (txStatus.isSuccess() && tx.getSleepingDevice() == false)) //Ждем ответного пакета от удаленного модема только если локальный ответ выключен или пакет отправлен в эфир и не предназначается спящему модему.
  //    getRemoteResponse(5);
  return;
}

void Command_To_Motor(int instruction)
{
  if (instruction == Corrupted_Command)
  {
    // NEED ADD REACTION On "ERROR_VALUE"
  }
  if (instruction == Stop)
  {
    if (SpeedValue_Now == ZeroPWM)
    {
      Step_For_Move = 0; // command already complete
    }

    if (SpeedValue_Now > ZeroPWM || SpeedValue_Now < -ZeroPWM )
    {
      Step_For_Move = 0; // pull to zero // 1 - FOR TESTS , 7 - for real fights
      SpeedValue_Now = ZeroPWM;
    }

    //    if (SpeedValue_Now > ZeroPWM)
    //    {
    //      Step_For_Move = -4; // pull to zero // 1 - FOR TESTS , 7 - for real fights
    //    }
    //    if (SpeedValue_Now < ZeroPWM)
    //    {
    //      Step_For_Move = 4; // pull to zero // 1 - FOR TESTS , 7 - for real fights
    //    }

    //Calculate
    SpeedValue_Now = SpeedValue_Now + Step_For_Move;
    //Teleport
    if ( ( (SpeedValue_Now <= ZeroPWM) & (SpeedValue_Now >= ZeroPWM - abs(Step_For_Move)) )                       //  (X <= 30) & (X >= 25)
         | ( (SpeedValue_Now >= (-1) * ZeroPWM) & (SpeedValue_Now <= ( (-1) * ZeroPWM + abs(Step_For_Move)) ) ) ) // ( X >= -30) & ( X <= -25 )
    {
      SpeedValue_Now = ZeroPWM;
      Step_For_Move = 0;  // command complete
      //PC_Debug.println("STOP NOW");
      digitalWrite(LedLimited, LOW);
    }
    //Debug
    PC_Debug.print("(Stopping)Speed == ");
    PC_Debug.println(SpeedValue_Now);
    PC_Debug.println("---------------");
  }
  if (instruction == Forward)
  {
    if (SpeedValue_Now == Max_SpeedValue) // check for Wished_Speed
    {
      //PC_Debug.println("Max Speed is reached! (already)");
      Step_For_Move = 0;
    }
    if (SpeedValue_Now >= Min_SpeedValue && SpeedValue_Now < Max_SpeedValue)
    {
      Step_For_Move = 4; //pull to 225 (Max_SpeedValue)
    }
    SpeedValue_Now = SpeedValue_Now + Step_For_Move;
    //Teleport
    if (SpeedValue_Now >= ZeroPWM * (-1) & (SpeedValue_Now <= ((-1) * ZeroPWM + abs(Step_For_Move)) ) )  // (X >= -30 & X <= -25 )
      SpeedValue_Now = ZeroPWM;
    // FINISHER
    if (SpeedValue_Now >= Max_SpeedValue & SpeedValue_Now <= ( Max_SpeedValue + abs(Step_For_Move) )) // ==Wished_Speed
    {
      SpeedValue_Now = Max_SpeedValue;
      Step_For_Move = 0;  //command complete
      digitalWrite(LedLimited, HIGH);
      //PC_Debug.println("Max Speed RIGHT NOW! LvL UP! ");
    }
    //Debug
    PC_Debug.print("(Grow) Speed == ");
    PC_Debug.println(SpeedValue_Now);
    PC_Debug.println("---------------");
  }
  //Backward
  if (instruction == Backward)
  {
    if (SpeedValue_Now == Min_SpeedValue)
    {
      //PC_Debug.println("Min Speed is reached! (already)");
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
      //PC_Debug.println("We Reversed");
    }
    //Debug
    PC_Debug.print("(Drop) Speed == ");
    PC_Debug.println(SpeedValue_Now);
  }
  Execute_The_Command(SpeedValue_Now);
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
      }
      else if (txStatus.getStatus() == TX_FAILURE_COMMAND_STATUS) // TX_FAILURE_COMMAND_STATUS == 4
      {
        //        PC_Debug.println("Пакет в эфир не ушел вследствие занятости канала");
      }
      else if (txStatus.getStatus() == ERROR_COMMAND_STATUS) // ERROR_COMMAND_STATUS == 1
      {
        //        PC_Debug.println("Недостаточно памяти для размещения пакета в буфере на передачу");
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
  PC_Debug.println("---------------------DEBUG------------------");
  //Print of Received Packet  Length
  PC_Debug.print("Packet Length -> ");
  PC_Debug.println(rx.getPacketLength());
  //Print of MSB Length
  PC_Debug.print("MSB Lenght -> ");
  PC_Debug.println(rx.getMsbLength());
  //Print of LSB Length
  PC_Debug.print("LSB Lenght -> ");
  PC_Debug.println(rx.getLsbLength());
  //Print of API ID
  PC_Debug.print("API ID -> ");
  PC_Debug.println(rx.getApiId());
  // Print Checksum
  PC_Debug.print("Checksum == ");
  PC_Debug.println(rx.getChecksum());
  // Print Frame Data Length
  PC_Debug.print("Frame Data Lenght -> ");
  PC_Debug.println(rx.getFrameDataLength());
  // Print Data Length
  PC_Debug.print("Data Length -> ");
  PC_Debug.println(rx.getDataLength());
  // Print RSSI
  PC_Debug.print("RSSI -> ");
  PC_Debug.println(rx.getRssi());

  PC_Debug.println("---------------------DEBUG------------------");
}

void Print_All_Array(int8_t Size, boolean WriteEnable)
{
  // ALL PACK PRINT!!
  for (int i = 0 ; i < Size ; i ++ )
  {
    //"PRINT"
    PC_Debug.print("Input testArray[");
    PC_Debug.print(i);
    PC_Debug.print("]= ");
    PC_Debug.println(rx.getData()[i]);
    //"WRITE"
    if (WriteEnable == true)
    {
      PC_Debug.print(" WRITE inside i -> ");
      PC_Debug.write(rx.getData()[i]);
      PC_Debug.println();
    }
  }
}

void EmergencyStop()
{
  cli();
  Alarm_ON();
  // STOP RIGHT NOW
  SpeedValue_Now = ZeroPWM;
  Step_For_Move = 0;
  digitalWrite(Direction_Of_Move, LOW);
  digitalWrite(Permission_Of_Move, LOW);
  digitalWrite(Permission_Of_Move, LOW);
  analogWrite(PWM_Pin, ZeroPWM);
  sei();
}

void ADCread() {
  Current_1 = analogRead(ADCpin_Current1);
  Speed_1 = analogRead(ADCpin_Speed1);
  return;
}

void setArrayForTelemetry() {
  int Direction = 111;
  if (SpeedValue_Now == 30 || SpeedValue_Now == -30)
    Direction = 0; //stop
  if (SpeedValue_Now > 30)
    Direction = 5; //rush
  if (SpeedValue_Now < -30)
    Direction = 2; //back
  //set values
  testArray [0] = CounterOfPacketsFromTx;
  testArray [1] = Speed_1;       // Speed_1L
  testArray [2] = Speed_1 >> 8;  // Speed_1L
  testArray [3] = Speed_2;
  testArray [4] = Speed_2 >> 8;
  testArray [5] = Current_1;
  testArray [6] = Current_1 >> 8;
  testArray [7] = Current_2;
  testArray [8] = Current_2 >> 8;
  testArray [9] = 4;               // V_Roper L
  testArray [10] = 0 >> 8;         // V_Roper_H
  testArray [11] = abs(SpeedValue_Now); //PWM_value
  testArray [12] = Direction;      //Direction
  //  testArray [13] = Direction;
  return;
}

void Data_Send_To_Processing() {
  int Garbage = random(-1023, 1023);
  PC_Debug.print("Speed_1=");
  PC_Debug.print(Speed_1);
  PC_Debug.print(" ");

  PC_Debug.print("Speed_2=");
  PC_Debug.print(Speed_2);
  PC_Debug.print(" ");

  PC_Debug.print("I_1=");
  PC_Debug.print(Current_1);
  PC_Debug.print(" ");

  PC_Debug.print("I_2=");
  PC_Debug.print(Current_2);
  PC_Debug.print(" ");

  //  Serial.println(val);
  PC_Debug.print("Garbage=");
  PC_Debug.print(Garbage);
  PC_Debug.print(" ");
  PC_Debug.println();
  delay(30);
}
