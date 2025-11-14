import processing.serial.*;

Serial port1, port2, port3;

// 表示用変数
String name1 = "Zumo1", name2 = "Zumo2", name3 = "Zumo3"; // ← 機体名
String mode1 = "UNKNOWN", mode2 = "UNKNOWN", mode3 = "UNKNOWN";
String color1 = "", color2 = "", color3 = "";
float dist1 = -1, dist2 = -1, dist3 = -1;
float heading1 = -1, heading2 = -1, heading3 = -1;
int motorL1 = 0, motorR1 = 0;
int motorL2 = 0, motorR2 = 0;
int motorL3 = 0, motorR3 = 0;
String myString1 = null, myString2 = null, myString3 = null;
int LF = 10;

//加速度用変数
float ax1 = 0, ay1 = 0, az1 = 0;
float ax2 = 0, ay2 = 0, az2 = 0;
float ax3 = 0, ay3 = 0, az3 = 0;

// 表示座標用定数
int tBaseY = 40;
int tDistY = 30;

// ボタン用変数（位置は setup() で初期化）
String currentColor = "Red";
int buttonW = 150;
int buttonH = 60;
int buttonX = 0;
int buttonY = 0;

void setup() {
  size(1200, 800);

  // ボタン位置を右下に設定
  buttonX = width - buttonW - 20;
  buttonY = height - buttonH - 20;

  try {
    port1 = new Serial(this, "COM4", 9600);
    port1.clear(); port1.bufferUntil(0x0a);
    println("COM4 connected");
  } catch (Exception e) {
    println("COM4 not available");
    port1 = null;
  }

  try {
    port2 = new Serial(this, "COM5", 9600);
    port2.clear(); port2.bufferUntil(0x0a);
    println("COM5 connected");
  } catch (Exception e) {
    println("COM5 not available");
    port2 = null;
  }

  try {
    port3 = new Serial(this, "COM6", 9600);
    port3.clear(); port3.bufferUntil(0x0a);
    println("COM6 connected");
  } catch (Exception e) {
    println("COM6 not available");
    port3 = null;
  }

  background(0);
}

void draw() {
  // 背景色の表示（右下）
  if (currentColor.equals("Red")) {
    fill(255, 100, 100);
  } else {
    fill(100, 100, 255);
  }
  rect(width/2, height/2, width/2, height/2);

  // Zumo1（左上）
  fill(0); rect(0, 0, width/2, height/2);
  fill(255); textSize(40);
  text(name1 + " Mode: " + mode1, 20, tBaseY);
  textSize(30);
  text("Distance: " + dist1 + " cm", 20, tBaseY + tDistY);
  text("Color: " + color1, 20, tBaseY + tDistY * 2);
  text("Heading: " + heading1 + "°", 20, tBaseY + tDistY * 3);
  text("Motor[L:" + motorL1 + " R:" + motorR1 + "]", 20, tBaseY + tDistY * 4);
  text("Accel[X:" + ax1 + " Y:" + ay1 + " Z:" + az1 + "]", 20, tBaseY + tDistY * 5);
  float roll1 = degrees(atan2(ay1, az1));
  float pitch1 = degrees(atan2(-ax1, sqrt(ay1*ay1 + az1*az1)));
  text("Roll: " + nf(roll1, 1, 1) + "°", 20, tBaseY + tDistY * 6);
  text("Pitch: " + nf(pitch1, 1, 1) + "°", 20, tBaseY + tDistY * 7);
  if (myString1 != null) {
    text("Raw: " + myString1, 20, tBaseY + tDistY * 8);
  }
  // === Roll / Pitch 可視化（Zumo1） ===
  pushMatrix();
  translate(300, 300);  // Zumo1 Roll表示位置（左上）
  rotate(radians(degrees(atan2(ay1, az1))));
  fill(200, 255, 200);
  rectMode(CENTER);
  rect(0, 0, 100, 20);  // 横長バーでRoll
  popMatrix();
  
  pushMatrix();
  translate(450, 300);  // Zumo1 Pitch表示位置（左上）
  rotate(radians(degrees(atan2(-ax1, sqrt(ay1*ay1 + az1*az1)))));
  fill(200, 200, 255);
  rectMode(CENTER);
  rect(0, 0, 20, 100);  // 縦長バーでPitch
  popMatrix();
  // ✅ 描画モードを元に戻す
rectMode(CORNER);


  // Zumo2（右上）
  fill(0); rect(width/2, 0, width/2, height/2);
  fill(255); textSize(40);
  text(name2 + " Mode: " + mode2, width/2 + 20, tBaseY);
  textSize(30);
  text("Distance: " + dist2 + " cm", width/2 + 20, tBaseY + tDistY);
  text("Color: " + color2, width/2 + 20, tBaseY + tDistY * 2);
  text("Heading: " + heading2 + "°", width/2 + 20, tBaseY + tDistY * 3);
  text("Motor[L:" + motorL2 + " R:" + motorR2 + "]", width/2 + 20, tBaseY + tDistY * 4);
  text("Accel[X:" + ax2 + " Y:" + ay2 + " Z:" + az2 + "]", width/2 + 20, tBaseY + tDistY * 5);
  float roll2 = degrees(atan2(ay2, az2));
  float pitch2 = degrees(atan2(-ax2, sqrt(ay2*ay2 + az2*az2)));
  text("Roll: " + nf(roll2, 1, 1) + "°", width/2 + 20, tBaseY + tDistY * 6);
  text("Pitch: " + nf(pitch2, 1, 1) + "°", width/2 + 20, tBaseY + tDistY * 7);
  if (myString2 != null) {
    text("Raw: " + myString2, width/2 + 20, tBaseY + tDistY * 8);
  }

  // Zumo3（左下）
  fill(0); rect(0, height/2, width/2, height/2);
  fill(255); textSize(40);
  text(name3 + " Mode: " + mode3, 20, height/2 + tBaseY);
  textSize(30);
  text("Distance: " + dist3 + " cm", 20, height/2 + tBaseY + tDistY);
  text("Color: " + color3, 20, height/2 + tBaseY + tDistY * 2);
  text("Heading: " + heading3 + "°", 20, height/2 + tBaseY + tDistY * 3);
  text("Motor[L:" + motorL3 + " R:" + motorR3 + "]", 20, height/2 + tBaseY + tDistY * 4);
  text("Accel[X:" + ax3 + " Y:" + ay3 + " Z:" + az3 + "]", 20, height/2 + tBaseY + tDistY * 5);
  float roll3 = degrees(atan2(ay3, az3));
  float pitch3 = degrees(atan2(-ax3, sqrt(ay3*ay3 + az3*az3)));
  text("Roll: " + nf(roll3, 1, 1) + "°", 20, height/2 + tBaseY + tDistY * 6);
  text("Pitch: " + nf(pitch3, 1, 1) + "°", 20, height/2 + tBaseY + tDistY * 7);
  if (myString3 != null) {
    text("Raw: " + myString3, 20, height/2 + tBaseY + tDistY * 8);
  }

  drawButton(); // ボタン描画
}

void drawButton() {
  if (currentColor.equals("Red")) {
    fill(255, 0, 0);
  } else {
    fill(0, 0, 255);
  }
  rect(buttonX, buttonY, buttonW, buttonH, 10);

  fill(255);
  textSize(20);
  textAlign(CENTER, CENTER);
  text(currentColor, buttonX + buttonW/2, buttonY + buttonH/2);
  textAlign(LEFT, BASELINE); // 他のテキスト描画に影響しないように戻す
}

void mousePressed() {
  if (mouseX > buttonX && mouseX < buttonX + buttonW &&
      mouseY > buttonY && mouseY < buttonY + buttonH) {
    // 色を切り替える
    if (currentColor.equals("Red")) {
      currentColor = "Blue";
    } else {
      currentColor = "Red";
    }
    // Arduinoに1文字だけ送信
    char signal = currentColor.equals("Red") ? 'R' : 'B';
    if (port1 != null) port1.write(signal);
    if (port2 != null) port2.write(signal);
    if (port3 != null) port3.write(signal);
  }
}

void serialEvent(Serial p) {
  String incoming = p.readStringUntil(LF);
  println("Received from " + p + ": " + incoming);

  if (incoming != null) {
    incoming = trim(incoming);

    if (incoming.startsWith("NAME:")) {
      String name = incoming.substring(5);
      if (p == port1) name1 = name;
      else if (p == port2) name2 = name;
      else if (p == port3) name3 = name;
    } else if (incoming.startsWith("MODE:")) {
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
    } else if (incoming.startsWith("ACCEL:")) {
      String[] parts = split(incoming.substring(6), ',');
      if (parts.length == 3) {
        float ax = float(parts[0]);
        float ay = float(parts[1]);
        float az = float(parts[2]);
        if (p == port1) {
          ax1 = ax; ay1 = ay; az1 = az;
        } else if (p == port2) {
          ax2 = ax; ay2 = ay; az2 = az;
        } else if (p == port3) {
          ax3 = ax; ay3 = ay; az3 = az;
        }
      }
    } else if (incoming.equals("REQUEST_COLOR")) {
      // ✅ Zumoから色リクエストが来たら送信
      char signal = currentColor.equals("Red") ? 'R' : 'B';
      if (p == port1) port1.write(signal);
      else if (p == port2) port2.write(signal);
      else if (p == port3) port3.write(signal);

    } else {
      if (p == port1) myString1 = incoming;
      else if (p == port2) myString2 = incoming;
      else if (p == port3) myString3 = incoming;
    }
  }
}
