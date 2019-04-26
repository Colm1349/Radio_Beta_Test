

import processing.serial.*;
//Variables for Graph Lines
int xn=0; // переменная координаты X начала
int yn=0; // переменная координаты Y начала 
int xk=0; // переменная координаты X конца
int yk=0; // переменная координаты Y конца 

//Data from Arduino
int Speed_1 = 1349;
int Speed_2 = 0;
int I_1 = 10;
int I_2 = 20;
int V_Roper = 0;
int V_control_panel = 0;
int PWM_Value = 3;
char Movement = 'S';
String Direction = "STOP";
int RSSI_Value = 0;
//int Voltage_1 = 1;
//int Voltage_2 = 2;

String inString;

Serial myPort;        // The serial port
float inByte = 0;

void setup() {
  size (900, 900); // устанавливаем размер окна
  background(0); // цвет заливки черный
  textSize(32); // размер текста
  //println(Serial.list()); 
  //for linux
  //  /dev/ttyS0 | /dev/ttyS1 | /dev/ttyS2 | /dev/ttyS3 | /dev/ttyUSB0
  //for Windows
  // ?????????????????????

  myPort = new Serial(this, "/dev/ttyUSB2", 74880);
  myPort.bufferUntil('\n'); //  Sets a specific byte to buffer until before calling serialEvent().
}

void draw() {

  //readSerial();
  //if ( ParseDataFromArduino() != 0 ) {
  //  println ("Corrupted Data from Arduino =(");
  //} else {
  //  //println("good parcel YEAH!");
  //}
  //ParseDataFromArduino();
  drawGraph();
}

void drawGraph() {

  yk=mouseY;  // пишем в переменную ук значение положения курсора мыши по оси Y
  int X_val = mouseX;
  int Y_val = mouseY;

  fill(0); // заливка для объекта прямоугольник
  noStroke();//рисовать прямоугольник без абриса
  rect (5, 5, 900, 180); //рисует прямоугольник перекрывающий наш текст
  fill(255); // заливка для текста
  //text("MouseY= "+Y_val+" pix", 10, 30); // пишем Y= и подставляем полученное значение  // I_1
  text("I1= "+I_1+" A", 10, 30);
  text("I2= "+I_2+" A", 10, 60);
  text("V_Roper= "+V_Roper+" V", 10, 90);
  text("V_Remote= "+V_control_panel+" V", 10, 120);
  text("Speed1 = "+Speed_1 + " RPM", 400, 30);
  text("Speed2 = "+Speed_2 + " RPM", 400, 60);
  text("PWM= "+PWM_Value+" %", 400, 90);
  text("Direction= "+Direction, 400, 120);
  text("RSSI_value= "+Y_val, 200, 150);  // RSSI_value
  stroke(255); // цвет будущей линии белый

  yk=(Speed_1)/2; // 1023/2 = 512. Cжатие масштаба x0.5 | X/4 Cжатие масштаба x0.25 ;
  line (xn+10, yn+200, xk+10, yk+200); // рисуем линию
  xn=xk; // после того как нарисовали линию присваиваем Xначала значение Хконца
  yn=yk; // после того как нарисовали линию присваиваем Yначала значение Yконца
  xk++; // смещаем Xконца на единицу
  if (xk > 850) { // reset Graph field
    xn=0; 
    yn=0;  
    xk=0; 
    yk=0;
    println("CLEAR ALL");
    clear();
  }
  //return ;
}

int ParseDataFromArduino() {  //return 0 if all good
  int EqualsIndex = 0;
  int EndLine = 0;
  int Counter_Value = 1;
  int Value_For_Flush = 0;
  int j = 1;
  System.out.println("Readed string from Serial port -> " + inString.length());
  for (int i = 0; i < inString.length(); i++)
  {
    if (inString.charAt(i) == '#' )
      break;

    if (inString.charAt(i) == '=' ) {
      EqualsIndex = i;
      System.out.println("EqualsIndex ==" + EqualsIndex);
    }
    if (inString.charAt(i) == ' ' ) {
      EndLine = i;
      System.out.println("EndLine ==" + EndLine);
      //String Convert into value
      String temp = inString.substring(EqualsIndex+1, EndLine); // [x, y)  y-1 deleted
      if (j == 8)
        System.out.println("Direction =="+ temp);
      System.out.println("temp[" + j + "] ==" + temp);
      System.out.println("temp[" + j + "].length ==" + temp.length());

      switch(j) {
      case 1:
        Speed_1 = Integer.parseInt(temp);
        break;
      case 2:
        Speed_2 = Integer.parseInt(temp);
        break;
      case 3:
        I_1 = Integer.parseInt(temp);
        break;
      case 4:
        I_2 = Integer.parseInt(temp);
        break;
      case 5:
        V_Roper = Integer.parseInt(temp);
        break;
      case 6:
        V_control_panel = Integer.parseInt(temp);
        break;
      case 7:
        PWM_Value = Integer.parseInt(temp);
        break;
      case 8:
        Direction = temp;
        break;
      case 9:
        RSSI_Value = Integer.parseInt(temp);
        break;

      default:
        System.out.println("ERROR");
        break;
      }
      j++;  
      if (j > 4)
      {
        System.out.println("All Received");   
        break;
      }

      //debug
      //String good = "84";
      //System.out.println("good.length =" + good.length());
      //Speed_1 = Integer.parseInt(temp);
      //System.out.println("Speed_1 == " + Speed_1);   

      //ParseDataFromArduino();
      //System.out.print("Speed_1 == ");
      //System.out.println(Speed_1);
    }
  }
  return 0;
}

void serialEvent (Serial myPort) {
  // get the ASCII string:  
  inString = myPort.readStringUntil('\n');
  print("InSring ===");
  println(inString);
  //inByte = float(myPort.readStringUntil('\n'));
  delay(20);
  //print("inByte ==");
  //println(inByte);

  //  if (inString != null) {
  //    // trim off any whitespace:
  //    inString = trim(inString);
  //    // convert to an int and map to the screen height:
  //    inByte = float(inString);
  //    //print("INBYTE ==");
  //    //println(inByte);
  //    inByte = map(inByte, 0, 1023, 0, height);
  //  }
  ParseDataFromArduino();
}
