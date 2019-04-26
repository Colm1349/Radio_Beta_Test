

import processing.serial.*;
//Variables for Graph Lines
int xn=0; // переменная координаты X начала
int yn=0; // переменная координаты Y начала 
int xk=0; // переменная координаты X конца
int yk=0; // переменная координаты Y конца 

//Data from Arduino
double Speed_1 = 1349;
double Speed_2 = 0;
double I_1 = 10;
double I_2 = 20;
double V_Roper = 0;
double V_Control_Panel = 0;
double PWM_Value = 3;
char Movement = 'S';
int DirectionInt = 0;
String DirectionText = "STOP";
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

  myPort = new Serial(this, "/dev/ttyUSB2", 115200);
  myPort.bufferUntil('#'); //  Sets a specific byte to buffer until before calling serialEvent().
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

  //float f4 = 1234.567;
  //println(nf(f4, 0, 2)); // 1234.57 - rounded up

  float I_1_F = (524 - (float)I_1) * 0.0125;
  float I_2_F = (524 - (float)I_2) * 0.0125*0;
  float Speed_1_F = (530 - (int)Speed_1) *16.67;
  float Speed_2_F = (524 - (int)Speed_2) *16.67*0;
  //println(I_1_F);
  text("I1= "+ nf(I_1_F, 0, 2) +" A" + "(" + I_1 + ")", 10, 30); // *0.0125
  text("I2= "+ nf(I_2_F, 0, 2) +" A" + "(" + I_2 + ")", 10, 60);
  text("V_Roper= "+ V_Roper +" V", 10, 90);
  text("V_Remote= "+ V_Control_Panel +" V", 10, 120);
  text("Speed1 = "+ nf(Speed_1_F, 0, 0) + " RPM" + "(" + Speed_1 + ")", 400, 30); // *16.67
  text("Speed2 = "+ nf(Speed_2_F, 0, 0) + " RPM" + "(" + Speed_2 + ")", 400, 60);
  text("PWM= "+ PWM_Value +" %", 400, 90);
  text("Direction= "+ DirectionText + " [" + DirectionInt + "]", 400, 120);
  text("RSSI_value= "+ Y_val, 200, 150);  // RSSI_value
  stroke(255); // цвет будущей линии белый

  //yk=((int)Speed_1)/36; // 1023/2 = 512. Cжатие масштаба x0.5 | X/32 Cжатие масштаба x0.03125 ;\
  //yk=((int)Speed_1)/4;I_1
  yk=((int)I_1)/2;

  line (xn+10, yn+300, xk+10, yk+300); // рисуем линию
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
    if (inString.charAt(i) == '~' ) {
      EndLine = i;
      System.out.println("EndLine ==" + EndLine);
      //String Convert into value
      String temp = inString.substring(EqualsIndex+1, EndLine); // [x, y)  y-1 deleted
      if (j == 8)
        System.out.println("Direction ==" + temp);
      System.out.println("temp[" + j + "] ==" + temp);
      System.out.println("temp[" + j + "].length ==" + temp.length());

      switch(j) {
      case 1:
        Speed_1 = Double.parseDouble(temp);
        //Speed_1= Float.parseFloat(temp);
        break;
      case 2:
        Speed_2 = Double.parseDouble(temp);
        break;
      case 3:
        I_1 = Double.parseDouble(temp);
        break;
      case 4:
        I_2 = Double.parseDouble(temp);
        break;
      case 5:
        V_Roper = Double.parseDouble(temp);
        break;
      case 6:
        PWM_Value = Double.parseDouble(temp);
        //V_control_panel = Integer.parseInt(temp);
        break;
      case 7:
        DirectionInt = Integer.parseInt(temp);
        if (DirectionInt == 0)
          DirectionText = "STOP";
        if (DirectionInt == 2)
          DirectionText = "BACK";
        if (DirectionInt == 5)
          DirectionText = "FORWARD";
        break;
      case 8:
        V_Control_Panel = Integer.parseInt(temp);
        break;
      case 9:
        RSSI_Value = Integer.parseInt(temp);
        break;

      default:
        System.out.println("____ERROR____");
        break;
      }
      j++;  
      if (j > 8)
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
  inString = myPort.readStringUntil('#');
  print("InSring ===");
  println(inString);
  //inByte = float(myPort.readStringUntil('\n'));

  //delay(20);
  delay(5);
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
