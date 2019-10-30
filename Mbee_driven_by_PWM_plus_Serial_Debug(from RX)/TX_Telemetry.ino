/**333
   с MBee-Arduino.
   MBee-Arduino является бесплатным программным обеспечением.
   Подробная информация о лицензиях находится в файле mbee.h.
   \author </i> von Boduen. Special thanx to Andrew Rapp.
*/

// Version 0.9


//Описание
/*  Пульт радиоуправления способный передать команду самоходу по радиуправлению через MBee или по кабелю
    и принять телеметрию , далее отобразить основные парамерты на дисплее.
    Имеет возможность активировать звуковую индикацию и посылать отладочные сообщения через пины 7 и 8 на PC
    Аварийный стоп + защита от одновременного зажатия ВПЕРЕД И НАЗАД

    Надо добавить
    1) Переключение режимов по проводу или по радио
    2) Настройка обработки массива данных от самохода по кабелю.
    3) Звуковую индикацию
    4) При отсутствиии телеметрии больше 0.5-4 сек перезагружаться.
    5) Чтение АЦПхой уровня напряжения батареи (кроны) и рассчитать резисторы.
    6) Отображение светодиодом режима работы (проводное/беспроводное)
   +7) работа с дисплеем

*/


#include <MBee.h>
#include <SoftwareSerial.h>
#include <LiquidCrystalRus.h>
#include <stdio.h>

#define DEBUG nss
#define MBee_Serial Serial
#define Stop 0
#define Forward 5
#define Backward 2
#define Corrupted_Command 10
#define EmergencyStopCode 170     // code for Stop device. (no breaks and just fall down)
#define ADC_Speed_RPM_Step 16.67
#define ADC_Current_Step 0.0125
#define ADC_Voltage_Step 0.0047
#define ADC_PWM_Step 0.4
#define StopButtonPin 10
#define EmergencyStopCode 52

uint8_t ArrayFromRX [] = {0, 11, 12, 21, 22, 31, 32, 41, 42, 51, 52, 120, 130};
uint8_t ssRX = 8;
uint8_t ssTX = 9;
SoftwareSerial nss(ssRX, ssTX);
SerialStar mbee = SerialStar();
LiquidCrystalRus disp(6, 7, 2, 3, 4, 5); // создаем объект

//TX SETUP
TxRequest tx = TxRequest(); //Пакет с данными для отправки удаленному модему. Для экономии памяти будем использовать один и тот же объект для отправки всех пакетов.
TxStatusResponse txStatus = TxStatusResponse(); //Локальный ответ со статусом команды.

//RX SETUP
RxResponse rx = RxResponse();
RxAcknowledgeResponse remoteRxStatus = RxAcknowledgeResponse(); //Пакет с подтверждением от удаленного модема получение данных.

//Variables
uint16_t remoteAddress = 0x0001; //Адрес модема, которому будут передаваться данные.
uint8_t testArray [] = {200, 100, 88};
uint8_t option = 0;
uint8_t data = 0;
int errorLed = 5;
int statusLed = 13; //Используется встроенный в Вашу плату Arduino светодиод. = LED_BUILTIN
int Green_Led = 11;
int Red_Led = 12;
int ForwardPin = 2;
int BackwardPin = 3;
int CommandForRaper = 0; // 0 == stop
bool lastButton = false;
bool currentButton = false;

//Telemetry members
float Speed_1_RPM = 0;
float Speed_2_RPM = 0;
float I_1_A = 0;
float I_2_A = 0;
float Battery_Roper_Votlage = 0;
float PWM_Value_Percent = 12.3;

//Display stuff

void setup()
{
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(Green_Led, OUTPUT);
  pinMode(Red_Led, OUTPUT);
  pinMode(ForwardPin , INPUT);
  pinMode(BackwardPin, INPUT);
  pinMode(StopButtonPin, INPUT);
  disp.begin(20, 4);  // инициализируем дисплей 4 строки по 20 символов
  reset_textmode();
  disp.clear();       // чистим дисплей
  DEBUG.begin(115200);
  MBee_Serial.begin(115200);
  mbee.begin(MBee_Serial);
  DEBUG.println("TX READY FOR your suffering!");
  tx.setRemoteAddress(remoteAddress); //Устанавливаем адрес удаленного модема.
  // write const on the display
  OLED_setup();
  delay(500); //Задержка не обязательна и вставлена для удобства работы с терминальной программой.
}

void loop()
{
  static int number_of_packet = 0;
  if (number_of_packet >= 16001) number_of_packet = 0;
  DEBUG.print("Sending...[");
  DEBUG.print(number_of_packet);
  DEBUG.println("]");
  //Read command
  ReadCommandFromSwither();
  //Check Button
  currentButton = debounce(lastButton);  // ???
  //currentButton = digitalRead(StopButtonPin);  // good
  if ( currentButton == true )
  {
    DEBUG.println("Emergency STOP THIS");
    testArray[2] = EmergencyStopCode;  // EmergencyStopCode 52
  }
  else
  {
    DEBUG.println("All HOPE IS OK");
    testArray[2] = 88;
  }
  //Write our command into "testArray" for send.
  testArray[0] = number_of_packet;
  testArray[1] = CommandForRaper;
  tx.setPayload((uint8_t*)testArray); //Устанавливаем указатель на тестовый массив
  tx.setPayloadLength(sizeof(testArray));  //Устанавливаем длину поля данных
  sendData();
  delay(50);
  number_of_packet++;     // +1 counter of packet
  //  DEBUG.print("tx.getDataLength() -- ");
  //  DEBUG.println(tx.getFrameDataLength());
  //  for ( int i = 0 ; i < tx.getFrameDataLength() ; i++)
  //  {
  //    DEBUG.print("tx.getFrameData(");
  //    DEBUG.print(i);
  //    DEBUG.print(") = ");
  //    DEBUG.println(tx.getFrameData(i));
  //  }

  //read input Telemetry
  mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
  if (mbee.getResponse().isAvailable())
  {
    DEBUG.println("TX Packet from Rx READED, NICE!");
    DEBUG.print("API ID === ");
    DEBUG.println( mbee.getResponse().getApiId() );
    // Print RSSI
    DEBUG.print("RSSI -> ");
    DEBUG.println(rx.getRssi());
    //    Debug_information_About_Rx_Packet(); // debug info
    if (mbee.getResponse().getApiId() == RECEIVE_PACKET_API_FRAME || mbee.getResponse().getApiId() == RECEIVE_PACKET_NO_OPTIONS_API_FRAME)
    {
      mbee.getResponse().getRxResponse(rx); //Получаем пакет с данными. //вписываем данные в "rx"
      if (rx.getDataLength() == 1) //Ожидаем, что передается только 1 байт.
      {
        data = rx.getData()[0]; //Считываем принятый байт данных.
        if ((data > '0') && (data <= '9')) //Проверяем, находится ли принятый байт в заданном диапазоне.
        {
          DEBUG.print("Value [");
          DEBUG.print("0");
          DEBUG.print("] -> ");
          DEBUG.println(rx.getData()[0]);
          flashLed(statusLed, data - '0', 200); //Моргаем светодиодом в соответствии с нажатой цифровой клавишей.
        }
      }
      else
      {
        DEBUG.println("TX Packet have many Symbols");
        int j = 0;
        //"WRITE" ALL PACKET
        if (rx.getDataLength() > 1)
        {
          DEBUG.println("TX Input Data :");
          for (int i = 0 ; i < rx.getDataLength() ; i++ )
          {
            //            if ( i >= 0) {
            //              DEBUG.print("Value [");
            //              DEBUG.print(i);
            //              DEBUG.print("] = ");
            //              DEBUG.println(rx.getData()[i]);
            //            }
            if ( i >= 8 ) {
              ArrayFromRX[j] = rx.getData()[i];
              //              DEBUG.print("ArrayFromRX [");
              //              DEBUG.print(i);
              //              DEBUG.print("] = ");
              //              DEBUG.println(ArrayFromRX[j]);
              j++;
            }
          }

          //          int arraySize = sizeof(ArrayFromRX) / sizeof(ArrayFromRX[0]);
          //          for (int i = 0; i < arraySize ; i++ )
          //          {
          //            DEBUG.print("ArrayFromRX_cutted [");
          //            DEBUG.print(i);
          //            DEBUG.print("] -->");
          //            DEBUG.println(ArrayFromRX[i]);
          //          }

        }
        Printing_Values_On_A_PC_Monitor();
      }
    }
    else
      DEBUG.println("Unexpected API ID from RX");
    //      flashLed(errorLed, 1, 500); //Принят фрейм, не предназначенный для передачи неструктурированных данных.

  }
  else
    DEBUG.println("---");

  Display_Data();
  //  DEBUG.println("///////////////end of loop////////////");
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
    DEBUG.println("Command RUSH ");
  }
  // Backward command
  if (Rush == false & Back == true)
  {
    CommandForRaper = Backward;
    digitalWrite(Red_Led,  HIGH);
    digitalWrite(Green_Led, LOW);
    DEBUG.println("Command BACK ");
  }
  // Stop command
  if (Rush == false & Back == false)
  {
    CommandForRaper = Stop;
    digitalWrite(Red_Led,  LOW);
    digitalWrite(Green_Led, LOW);
    DEBUG.println("Command STOP !!! ");
  }
  // Error
  if (Rush == true & Back == true)
  {
    DEBUG.println();
    DEBUG.println("I take Error from \"switcher\" , =( ");
    DEBUG.println();
    CommandForRaper = Stop;
    digitalWrite(Red_Led,  HIGH);
    digitalWrite(Green_Led, HIGH);
    digitalWrite(statusLed, HIGH);
    DEBUG.println(" NO Command, we have a ERROR ((( ");
  }
  DEBUG.print("CommandForRaper --> ");
  DEBUG.println(CommandForRaper);
}


void sendData()
{
  mbee.send(tx);
  //    if (tx.getFrameId()) //Проверяем, не заблокировано ли локальное подтверждение отправки.
  //      getLocalResponse(50);
  if ((tx.getFrameId() == 0) || (txStatus.isSuccess() && tx.getSleepingDevice() == false)) //Ждем ответного пакета от удаленного модема только если локальный ответ выключен или пакет отправлен в эфир и не предназначается спящему модему.
    getRemoteResponse(5);
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
    digitalWrite(pin, LOW);
    if (i + 1 < times)
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
    DEBUG.println("Emergency Stop == TRUE ");
    delay(20);
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
    DEBUG.println("last != current");
    current = digitalRead(StopButtonPin);
  }
  return current;
}

void Printing_Values_On_A_PC_Monitor()
{
  int temp = 0;
  DEBUG.print("Speed_1=");
  temp = ArrayFromRX[1] + (ArrayFromRX[2] << 8);
  Speed_1_RPM = (540 - temp) * ADC_Speed_RPM_Step; //because invertor after ADC
  //  DEBUG.print(Speed_1_RPM);
  DEBUG.print(temp);

  DEBUG.print("~Speed_2=");
  temp = ArrayFromRX[3] + (ArrayFromRX[4] << 8);
  Speed_2_RPM = (540 - temp) * ADC_Speed_RPM_Step;
  DEBUG.print(Speed_2_RPM);

  DEBUG.print("~I_1=");
  temp = ArrayFromRX[5] + (ArrayFromRX[6] << 8);
  I_1_A = (550 - temp) * ADC_Current_Step;
  DEBUG.print(temp);  //I_1_A

  DEBUG.print("~I_2=");
  temp = ArrayFromRX[7] + (ArrayFromRX[8] << 8);
  I_2_A = temp * ADC_Current_Step;
  DEBUG.print(I_2_A);
  DEBUG.print("~V_Roper=");
  temp = ArrayFromRX[9] + (ArrayFromRX[10] << 8);
  Battery_Roper_Votlage = temp * ADC_Voltage_Step;
  DEBUG.print(Battery_Roper_Votlage);
  DEBUG.print("~PWM_Value=");
  PWM_Value_Percent = ArrayFromRX[11] * ADC_PWM_Step;
  DEBUG.print(PWM_Value_Percent);
  DEBUG.print("~Direction=");
  DEBUG.print(ArrayFromRX[12]);
  //REWORK this V_Control_Panel !!!!
  int V_Control_Panel = digitalRead(0);
  DEBUG.print("~V_Control_Panel=");
  DEBUG.print("1388");
  // Print RSSI
  DEBUG.print("~RSSI=");
  DEBUG.print(rx.getRssi());

  //  DEBUG.print( V_Control_Panel );
  DEBUG.println("~ #");
  DEBUG.println();
  delay(30);
  return;
}

void Display_Data()
{
  //mute blink
  disp.noBlink();
  byte Triangle0[8] = {
    0b11111,
    0b01110,
    0b00100,
    0b00100,
    0b00100,
    0b00000,
    0b00001,
    0b00111
  };

  byte Triangle1[8] = {
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00001,
    0b00111,
    0b11111,
    0b11111
  };

  byte Triangle2[8] = {
    0b00000,
    0b00000,
    0b00001,
    0b00111,
    0b11111,
    0b11111,
    0b11111,
    0b11111
  };

  byte Triangle3[8] = {
    0b00001,
    0b00111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111
  };

  byte RC_Char[8] = {
    0b01000,
    0b01000,
    0b11111,
    0b10001,
    0b10001,
    0b11111,
    0b11101,
    0b11111
  };
  byte Roper[8] = {
    0b11111,
    0b11011,
    0b10001,
    0b00100,
    0b10001,
    0b11011,
    0b11111,
    0b11111
  };

  //Create Custom Sign
  //  disp.createChar(0, crossInverted);
  //  disp.createChar(1, cross);
  disp.createChar(2, Roper);
  disp.createChar(3, RC_Char);
  disp.createChar(4, Triangle0);
  disp.createChar(5, Triangle1);
  disp.createChar(6, Triangle2);
  disp.createChar(7, Triangle3);

  //Telemetry Emulation
  static int S1; //random(-101, 101);
  static int S2;
  static int CUR1;
  static int CUR2;
  static int Bat_Climb;
  static int Bat_RC;
  static int RSSI_LvL;
  //  S1 = random(-110, 110); //random(-101, 101);
  //  S2 = random(-120, 122);
  //  CUR1 = random(-110, 110);
  //  CUR2 = random(-120, 120);
  //  Bat_Climb = random(0, 100);
  //  Bat_RC = random(0, 100);
  //  RSSI_LvL = random(200, 220);

  S1 = 90;
  S2 = 90;
  CUR1 = 20;
  CUR2 = 20;
  Bat_Climb = 97;
  Bat_RC = 100;
  RSSI_LvL = 230;

  int offset = 0;
  int LvL = 0;
  Calculate_Speed(&S1, &S2);
  Calculate_Current(&CUR1, &CUR2);
  Calculate_BatteryLevel(&Bat_Climb, &Bat_RC);

  //RSSI processing
  disp.setCursor(5, 0);
  disp.print("   ");
  //  disp.setCursor(5, 0);
  //  disp.print(RSSI_LvL);
  if (RSSI_LvL >= 0 & RSSI_LvL < 32)
    LvL = 1;
  if (RSSI_LvL >= 32 & RSSI_LvL < 64)
    LvL = 2;
  if (RSSI_LvL >= 64 & RSSI_LvL < 96)
    LvL = 3;
  if (RSSI_LvL >= 96 & RSSI_LvL < 128)
    LvL = 4;
  if (RSSI_LvL >= 128 & RSSI_LvL < 160)
    LvL = 5;
  if (RSSI_LvL >= 160 & RSSI_LvL < 192)
    LvL = 6;
  if (RSSI_LvL >= 192 & RSSI_LvL < 224)
    LvL = 7;
  if (RSSI_LvL >= 224 & RSSI_LvL < 255)
    LvL = 8;

  String buffer1 = "-100";
  String buffer2 = "-200";
  //Second line
  buffer1 = Compiler_for_OLED (S1);
  buffer2 = Compiler_for_OLED (S2);
  disp.setCursor(5, 1);
  disp.print(buffer1);
  disp.setCursor(12, 1);
  disp.print(buffer2);
  //Third line
  buffer1 = Compiler_for_OLED (CUR1);
  buffer2 = Compiler_for_OLED (CUR2);
  disp.setCursor(5, 2);
  disp.print(buffer1);
  disp.setCursor(12, 2);
  disp.print(buffer2);
  //Fourth line
  buffer1 = Compiler_Battery_LvL_for_OLED(Bat_Climb);
  buffer2 = Compiler_Battery_LvL_for_OLED(Bat_RC);
  disp.setCursor(6, 3);
  disp.print(buffer1);
  disp.setCursor(13, 3);
  disp.print(buffer2);
  //  int minusVal = random (-100, 100);
  //  String small = "-100";
  RSSI_plot(LvL);
  delay(1000);
  return;
}

String Compiler_Battery_LvL_for_OLED(int input)
{
  String small = "100";
  if (input >= 0)
  {
    small = String(input);
    Serial.println("battery > 0");
    Serial.println(small);
    if (input >= 0 & input < 10)
      small = "  " + String(input);
    if (input >= 10 & input < 100)
      small = " " + String(input);
    if (input >= 100)
      small = String(input);
    //
    Serial.println(small);
    delay(1);
    return small;
  }
  else
  return "ERR";

}

String Compiler_for_OLED(int input)
{
  String small = "-100";
  // compile the buffer
  if (input < 0)
  {
    small = String(input);
    Serial.println("input < 0");
    //    Serial.println("We've destroyed this world / Darkness fills the sky");
    Serial.println(small);
    if (input < 0 & input > -10)
      small = "  " + String(input);
    if (input <= -10 & input > -100)
      small = " " + String(input);
    if (input <= -100)
      small = String(input);
    //
    Serial.println(small);
    delay(5);
  }
  else // minus val >= 0
  {
    small = String(input);
    Serial.println("input > 0");
    //    Serial.println("You don't know my misery!");
    Serial.println(small);
    if (input >= 0 & input < 10)
      small = "   " + String(input);
    if (input >= 10 & input < 100)
      small = "  " + String(input);
    if (input >= 100)
      small = " " + String(input);
    //
    Serial.println(small);
    delay(5);
  }
  return small;
}

void OLED_setup()
{
  //Zero line
  //  disp.setCursor(0, 0);
  //  disp.print("TEST");

  //Second line
  disp.setCursor(0, 1);
  disp.print("SPD ");
  disp.setCursor(9, 1);
  disp.print("%");
  disp.setCursor(16, 1);
  disp.print("%");

  //Third line
  disp.setCursor(0, 2);
  disp.print("LOAD");
  disp.setCursor(9 , 2);
  disp.print("%");
  disp.setCursor(16 , 2);
  disp.print("%");

  //Fourth line
  disp.setCursor(0, 3);
  disp.print("BAT");
  disp.setCursor(5, 3);
  disp.write(byte(2));  //symbol Climber
  disp.setCursor(9, 3);
  disp.print("%");
  disp.setCursor(12, 3);
  disp.write(byte(3));  //symbol RC
  disp.setCursor(16, 3);
  disp.print("%");
  return;
}

// Plot
void RSSI_plot(int LvL)
{
  if (LvL == 1 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.print("   ");
    disp.setCursor(16, 0);
    //    disp.blink();
  }
  if (LvL == 2 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.print("   ");
  }
  if (LvL == 3 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.print("  ");
    disp.setCursor(17, 0);
    //    disp.blink();
  }
  if (LvL == 4 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.print("  ");
  }
  if (LvL == 5 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.print(" ");
    disp.setCursor(18, 0);
    //    disp.blink();
  }
  if (LvL == 6 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.print(" ");
  }
  if (LvL == 7 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.write(byte(7));
    disp.setCursor(19, 0);
    //    disp.blink();
  }
  if (LvL == 8 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.write(byte(7));
  }
  return;
}

void Calculate_Speed(int S1, int S2)
{
  S1 = random(50, 1000);
  Serial.print("Random S1 ->");
  Serial.println(S1);
  S1 = map(S1, 50, 1000, -100, 100);
  Serial.print("Speed 1 ->");
  Serial.println(S1);
  S2 = random(50, 1000);
  Serial.print("Random S2 ->");
  Serial.println(S2);
  S2 = map(S2, 50, 1000, -100, 100);
  Serial.print("Speed 2 ->");
  Serial.println(S2);
  return;
}

void Calculate_Current(int CUR1, int CUR2)
{
  CUR1 = random(50, 1000);
  Serial.print("Random CUR1 ->");
  Serial.println(CUR1);
  CUR1 = map(CUR1, 50, 1000, -100, 100);
  Serial.print("CUR 1 ->");
  Serial.println(CUR1);
  CUR2 = random(50, 1000);
  Serial.print("Random CUR2 ->");
  Serial.println(CUR2);
  CUR2 = map(CUR2, 50, 1000, -100, 100);
  Serial.print("CUR 2 ->");
  Serial.println(CUR2);
  return;
}

void Calculate_BatteryLevel(int Bat_Climb, int Bat_RC)
{
  Bat_Climb = random(50, 1000);
  Serial.print("Random Batt Climb ->");
  Serial.println(Bat_Climb);
  Bat_Climb = map(Bat_Climb, 50, 1000, 0, 100);
  Serial.print("Batt Climb ->");
  Serial.println(Bat_Climb);
  Bat_RC = random(50, 1000);
  Serial.print("Random Bat Rc ->");
  Serial.println(Bat_RC);
  Bat_RC = map(Bat_RC, 50, 1000, 0, 100);
  Serial.print("Bat_RC ->");
  Serial.println(Bat_RC);
  return;
}

void reset_textmode() //функция для установки графического режима
{
  disp.command(0x08);//выключили экран
  disp.command(0x17);//переключение в текстовый режим
  disp.command(0x01);//очистили от мусора ОЗУ
  disp.command(0x04 | 0x08);//включили экран
}
