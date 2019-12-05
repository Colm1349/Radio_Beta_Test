void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);

  enterCommandMode();
  readSerialLine();
  delay(1000);
  Serial.println("sending command");
  sendCommand("atap6");
  delay(1000);
  readSerialLine();
}

void switchCommandMode() {
  Serial2.print("+");
  delay(10);
  Serial2.print("+");
  delay(10);
  Serial2.print("+");
  delay(10);
}

void enterCommandMode() {
  switchCommandMode();
  if (Serial2.read() == 13){
    Serial.println("Entered command mode");
    return;
  } else {
    Serial.println("Left command mode");
    switchCommandMode();
    Serial.println("Entered command mode");
  }
}

void sendCommand(char* command) {
  Serial2.println(command);
}

void readSerialLine() {
  
  int incomingByte = 0;
  while (Serial2.available() > 0) {  //если есть доступные данные
    // считываем байт
    incomingByte = Serial2.read();

    // отсылаем то, что получили
    Serial.print("I received: ");
    Serial.write(incomingByte);
    Serial.write(10);
  }
}

void loop() {
  // put your main code here, to run repeatedly:


}
