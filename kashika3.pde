import processing.serial.*;

Serial port1;
Serial port2;
Serial port3;

//表示用変数
String mode1 = "UNKNOWN", mode2 = "UNKNOWN";
String color1 = "", color2 = "";
float dist1 = -1, dist2 = -1;
float heading1 = -1, heading2 = -1;
int motorL1 = 0, motorR1 = 0;
int motorL2 = 0, motorR2 = 0;
String myString1 = null, myString2 = null;
int LF = 10;

//表示座標用定数
int tBaseY = 40;
int tDistY = 30;

void setup() {
  size(1200, 800);
  port1 = new Serial(this, "COM4", 9600);
  port1.clear();
  port1.bufferUntil(0x0d);

  port2 = new Serial(this, "COM5", 9600);
  port2.clear();
  port2.bufferUntil(0x0d);
  
  port3 = new Serial(this, "COM6", 9600);
  port3.clear();
  port3.bufferUntil(0x0d);
  

  background(0);
  fill(100); rect(width/2, 0, width/2, height/2);
  fill(200); rect(0, height/2, width/2, height/2);
  fill(150); rect(width/2, height/2, width/2, height/2);
}

void draw() {
  fill(0); rect(0, 0, width/2, height/2);
  fill(255); textSize(40);
  text("Zumo1 Mode: " + mode1, 20, tBaseY);
  textSize(30);
  text("Distance: " + dist1 + " cm", 20, tBaseY + tDistY);
  text("Color: " + color1, 20, tBaseY + tDistY * 2);
  text("Heading: " + heading1 + "°", 20, tBaseY + tDistY * 3);
  text("Motor[L:" + motorL1 + " R:" + motorR1 + "]", 20, tBaseY + tDistY * 4);
  if (myString1 != null) {
    text("Raw: " + myString1, 20, tBaseY + tDistY * 5);
  }

  fill(0); rect(width/2, 0, width/2, height/2);
  fill(255); textSize(40);
  text("Zumo2 Mode: " + mode2, width/2 + 20, tBaseY);
  textSize(30);
  text("Distance: " + dist2 + " cm", width/2 + 20, tBaseY + tDistY);
  text("Color: " + color2, width/2 + 20, tBaseY + tDistY * 2);
  text("Heading: " + heading2 + "°", width/2 + 20, tBaseY + tDistY * 3);
  text("Motor[L:" + motorL2 + " R:" + motorR2 + "]", width/2 + 20, tBaseY + tDistY * 4);
  if (myString2 != null) {
    text("Raw: " + myString2, width/2 + 20, tBaseY + tDistY * 5);
  }
  
  fill(0); rect(width/2, 0, width/2, height/2);
  fill(255); textSize(40);
  text("Zumo3 Mode: " + mode2, width/2 + 20, tBaseY);
  textSize(30);
  text("Distance: " + dist2 + " cm", width/2 + 20, tBaseY + tDistY);
  text("Color: " + color2, width/2 + 20, tBaseY + tDistY * 2);
  text("Heading: " + heading2 + "°", width/2 + 20, tBaseY + tDistY * 3);
  text("Motor[L:" + motorL2 + " R:" + motorR2 + "]", width/2 + 20, tBaseY + tDistY * 4);
  if (myString2 != null) {
    text("Raw: " + myString2, width/2 + 20, tBaseY + tDistY * 5);
  }
}

void serialEvent(Serial p) {
  String incoming = p.readStringUntil(LF);
  if (incoming != null) {
    incoming = trim(incoming);

    // デバッグ表示（必要に応じて有効化）
    // println((p == port1 ? "Zumo1" : "Zumo2") + " >> " + incoming);

    if (incoming.startsWith("MODE:")) {
      String modeStr = incoming.substring(5);
      if (p == port1) mode1 = modeStr;
      else mode2 = modeStr;
    } else if (incoming.startsWith("DIST:")) {
      float d = float(incoming.substring(5));
      if (p == port1) dist1 = d;
      else dist2 = d;
    } else if (incoming.startsWith("COLOR:")) {
      String c = incoming.substring(6);
      if (p == port1) color1 = c;
      else color2 = c;
    } else if (incoming.startsWith("HEADING:")) {
      float h = float(incoming.substring(8));
      if (p == port1) heading1 = h;
      else heading2 = h;
    } else if (incoming.startsWith("MOTOR:")) {
      String[] parts = split(incoming.substring(6), ',');
      if (parts.length == 2) {
        int ml = int(parts[0]);
        int mr = int(parts[1]);
        if (p == port1) {
          motorL1 = ml;
          motorR1 = mr;
        } else {
          motorL2 = ml;
          motorR2 = mr;
        }
      }
    } else {
      // その他のログを保存（任意）
      if (p == port1) myString1 = incoming;
      else myString2 = incoming;
    }
  }
}
