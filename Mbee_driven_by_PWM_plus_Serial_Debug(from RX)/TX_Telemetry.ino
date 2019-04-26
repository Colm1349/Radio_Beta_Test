/**333
   Этот файл является частью библиотеки MBee-Arduino.
   MBee-Arduino является бесплатным программным обеспечением.
   Подробная информация о лицензиях находится в файле mbee.h.
   \author </i> von Boduen. Special thanx to Andrew Rapp.
*/

// Version 0.9

#include <MBee.h>
#include <SoftwareSerial.h>

#define DEBUG nss
#define MBee_Serial Serial
#define Stop 0
#define Forward 5
#define Backward 2
#define Corrupted_Command 10
#define EmergencyStopCode 170
#define StopButtonPin 7
#define ADC_Speed_RPM_Step 16.67
#define ADC_Current_Step 0.0125
#define ADC_Voltage_Step 0.0047
#define ADC_PWM_Step 0.4

uint8_t ArrayFromRX [] = {0, 11, 12, 21, 22, 31, 32, 41, 42, 51, 52, 120, 130};
uint8_t ssRX = 8;
uint8_t ssTX = 9;
SoftwareSerial nss(ssRX, ssTX);
SerialStar mbee = SerialStar();

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
int errorLed = 56;
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

void setup()
{
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(Green_Led, OUTPUT);
  pinMode(Red_Led, OUTPUT);
  pinMode(ForwardPin , INPUT);
  pinMode(BackwardPin, INPUT);
  DEBUG.begin(115200);
  MBee_Serial.begin(115200);
  mbee.begin(MBee_Serial);
  DEBUG.println("TX READY FOR your suffering!");
  tx.setRemoteAddress(remoteAddress); //Устанавливаем адрес удаленного модема.
  delay(500); //Задержка не обязательна и вставлена для удобства работы с терминальной программой.
}

void loop()
{
  static int number_of_packet = 0;
  DEBUG.print("Sending...[");
  DEBUG.print(number_of_packet);
  DEBUG.println("]");
  //Read command
  ReadCommandFromSwither();
  //Write our command into "testArray" for send.
  testArray[0] = number_of_packet;
  testArray[1] = CommandForRaper;
  tx.setPayload((uint8_t*)testArray); //Устанавливаем указатель на тестовый массив
  tx.setPayloadLength(sizeof(testArray));  //Устанавливаем длину поля данных

  //Check Button
  currentButton = debounce(lastButton);
  if ( currentButton == true )
  {
    DEBUG.println("STOP THIS");
    delay(10);
    testArray[2] = EmergencyStopCode;
  }
  else
  {
    DEBUG.println("All HOPE IS OK");
    testArray[2] = 88;
  }

  sendData();
  delay(50);
  number_of_packet++;     // +1 counter of packet
  //read input Telemetry
  mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
  if (mbee.getResponse().isAvailable())
  {
    DEBUG.println("TX Packet READED, NICE!");
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
        //DEBUG.println("TX Packet have many Symbols");
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
          int arraySize = sizeof(ArrayFromRX) / sizeof(ArrayFromRX[0]);
          for (int i = 0; i < arraySize ; i++ )
          {
            DEBUG.print("ArrayFromRX_cutted [");
            DEBUG.print(i);
            DEBUG.print("] -->");
            DEBUG.println(ArrayFromRX[i]);
          }
        }
        Printing_Values_On_A_PC_Monitor();
      }
    }
    else
      flashLed(errorLed, 1, 500); //Принят фрейм, не предназначенный для передачи неструктурированных данных.

  }
  else
    DEBUG.println("---");
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
    DEBUG.println(" NO Command , we have a ERROR ((( ");
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
    DEBUG.println("Emergency Stop  == TRUE ");
    delay(200);
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
  //  DEBUG.print( V_Control_Panel );
  DEBUG.println("~ #");
  DEBUG.println();
  delay(30);
  return;
}
