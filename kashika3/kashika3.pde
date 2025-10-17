import processing.serial.*;

Serial port1, port2, port3;

// 表示用変数
String mode1 = "UNKNOWN", mode2 = "UNKNOWN", mode3 = "UNKNOWN";
String color1 = "", color2 = "", color3 = "";
float dist1 = -1, dist2 = -1, dist3 = -1;
float heading1 = -1, heading2 = -1, heading3 = -1;
int motorL1 = 0, motorR1 = 0;
int motorL2 = 0, motorR2 = 0;
int motorL3 = 0, motorR3 = 0;
String myString1 = null, myString2 = null, myString3 = null;
int LF = 10;

// 表示座標用定数
int tBaseY = 40;
int tDistY = 30;

void setup() {
  size(1200, 800);

  try {
    port1 = new Serial(this, "COM4", 9600);
    port1.clear(); port1.bufferUntil(0x0d);
    println("COM4 connected");
  } catch (Exception e) {
    println("COM4 not available");
    port1 = null;
  }

  try {
    port2 = new Serial(this, "COM5", 9600);
    port2.clear(); port2.bufferUntil(0x0d);
    println("COM5 connected");
  } catch (Exception e) {
    println("COM5 not available");
    port2 = null;
  }

  try {
    port3 = new Serial(this, "COM6", 9600);
    port3.clear(); port3.bufferUntil(0x0d);
    println("COM6 connected");
  } catch (Exception e) {
    println("COM6 not available");
    port3 = null;
  }

  background(0);
  fill(100); rect(width/2, 0, width/2, height/2);
  fill(200); rect(0, height/2, width/2, height/2);
  fill(150); rect(width/2, height/2, width/2, height/2);
}


void draw() {
  // Zumo1（左上）
  fill(0); rect(0, 0, width/2, height/2);
  fill(255); textSize(40);
  text("Zumo18 Mode: " + mode1, 20, tBaseY);
  textSize(30);
  text("Distance: " + dist1 + " cm", 20, tBaseY + tDistY);
  text("Color: " + color1, 20, tBaseY + tDistY * 2);
  text("Heading: " + heading1 + "°", 20, tBaseY + tDistY * 3);
  text("Motor[L:" + motorL1 + " R:" + motorR1 + "]", 20, tBaseY + tDistY * 4);
  if (myString1 != null) {
    textSize(30);
    text("Raw: " + myString1, 20, tBaseY + tDistY * 5);
  }

  // Zumo2（右上）
  fill(0); rect(width/2, 0, width/2, height/2);
  fill(255); textSize(40);
  text("Zumo201 Mode: " + mode2, width/2 + 20, tBaseY);
  textSize(30);
  text("Distance: " + dist2 + " cm", width/2 + 20, tBaseY + tDistY);
  text("Color: " + color2, width/2 + 20, tBaseY + tDistY * 2);
  text("Heading: " + heading2 + "°", width/2 + 20, tBaseY + tDistY * 3);
  text("Motor[L:" + motorL2 + " R:" + motorR2 + "]", width/2 + 20, tBaseY + tDistY * 4);
  if (myString2 != null) {
    text("Raw: " + myString2, width/2 + 20, tBaseY + tDistY * 5);
  }

  // Zumo3（左下）
  fill(0); rect(0, height/2, width/2, height/2);
  fill(255); textSize(40);
  text("Zumo204 Mode: " + mode3, 20, height/2 + tBaseY);
  textSize(30);
  text("Distance: " + dist3 + " cm", 20, height/2 + tBaseY + tDistY);
  text("Color: " + color3, 20, height/2 + tBaseY + tDistY * 2);
  text("Heading: " + heading3 + "°", 20, height/2 + tBaseY + tDistY * 3);
  text("Motor[L:" + motorL3 + " R:" + motorR3 + "]", 20, height/2 + tBaseY + tDistY * 4);
  if (myString3 != null) {
    text("Raw: " + myString3, 20, height/2 + tBaseY + tDistY * 5);
  }
}

void serialEvent(Serial p) {
  String incoming = p.readStringUntil(LF);
  if (incoming != null) {
    incoming = trim(incoming);

    if (incoming.startsWith("MODE:")) {
      String modeStr = incoming.substring(5);
      if (p == port1) mode1 = modeStr;
      else if (p == port2) mode2 = modeStr;
      else if (p == port3) mode3 = modeStr;
    } else if (incoming.startsWith("DIST:")) {
      float d = float(incoming.substring(5));
      if (p == port1) dist1 = d;
      else if (p == port2) dist2 = d;
      else if (p == port3) dist3 = d;
    } else if (incoming.startsWith("COLOR:")) {
      String c = incoming.substring(6);
      if (p == port1) color1 = c;
      else if (p == port2) color2 = c;
      else if (p == port3) color3 = c;
    } else if (incoming.startsWith("HEADING:")) {
      float h = float(incoming.substring(8));
      if (p == port1) heading1 = h;
      else if (p == port2) heading2 = h;
      else if (p == port3) heading3 = h;
    } else if (incoming.startsWith("MOTOR:")) {
      String[] parts = split(incoming.substring(6), ',');
      if (parts.length == 2) {
        int ml = int(parts[0]);
        int mr = int(parts[1]);
        if (p == port1) {
          motorL1 = ml; motorR1 = mr;
        } else if (p == port2) {
          motorL2 = ml; motorR2 = mr;
        } else if (p == port3) {
          motorL3 = ml; motorR3 = mr;
        }
      }
    } else {
      if (p == port1) myString1 = incoming;
      else if (p == port2) myString2 = incoming;
      else if (p == port3) myString3 = incoming;
    }
  }
}
