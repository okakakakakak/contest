#include "definitions.h"

// ============================================
// グローバルオブジェクト定義
// ============================================
Pushbutton button(ZUMO_BUTTON);
MotorController motor_ctrl;
ColorSensorState color_sensor;
CompassState compass_state;
UltrasonicSensor ultrasonic(2, 4);
RobotState robot_state;
PIController pi_ctrl;
int ACCEL_Z_OFFSET = 0;

// ============================================
// 定数定義
// ============================================
const float TARGET_HEADING = 210.0;
const float MAGNETIC_DECLINATION = -7.0;
const char ROBOT_NAME[] PROGMEM = "Zumo1";  // ← ロボット名（必要に応じて変更）

// ============================================
// ボタン待機関数（簡略版）
// ============================================
void waitForButtonPress() {
  Serial.println(F("Press button..."));
  
  while (!button.isPressed()) {
    delay(50);
  }
  
  while (button.isPressed()) {
    delay(10);
  }
  
  Serial.println(F("OK!"));
  delay(300);
}

// ============================================
// セットアップ
// ============================================
void setup() {
  Serial.begin(9600);  // 9600じゃないと速すぎて認識しないかも
  delay(1500);

  
  Serial.println(F("\n=== Zumo v3.1 ==="));
  
  // カラーセンサー初期化
  Serial.print(F("Color sensor..."));
  if (color_sensor.tcs.begin()) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("FAIL"));
    while (1) delay(1000);
  }
  
  // 地磁気センサー初期化
  Serial.print(F("Compass..."));
  compass_state.compass.init();
  compass_state.compass.enableDefault();
  compass_state.compass.m_min.x = -32767;
  compass_state.compass.m_min.y = -32767;
  compass_state.compass.m_max.x = 32767;
  compass_state.compass.m_max.y = 32767;
  Serial.println(F("OK"));
  
  // 超音波センサー初期化
  Serial.print(F("Ultrasonic..."));
  ultrasonic.init();
  Serial.println(F("OK"));
  
  // コンパスキャリブレーション
  Serial.println(F("--- Compass Calib ---"));
  waitForButtonPress();
  calibrationCompassAdvanced();
  Serial.println(F("Done!"));

  // 💡 NEW: Z軸オフセットキャリブレーション
  Serial.println(F("--- Accel Z Calib ---"));
  Serial.println(F("Place robot on a level surface. Press button."));
  waitForButtonPress();
  calibrateAccelZOffset();
  Serial.println(F("Done! Z-Offset: "));
  Serial.println(ACCEL_Z_OFFSET);
  
  // カラーキャリブレーション
  Serial.println(F("--- Color Calib ---"));
  waitForButtonPress();
  color_sensor.calibrate();
  Serial.println(F("Done!"));

  // 動作開始
  Serial.println(F("--- Start ---"));
  waitForButtonPress();

  robot_state.mode = STATE_INIT;
  robot_state.time_now = robot_state.time_prev = millis();
  
  Serial.println(F("Running..."));

  Serial.println("NAME:AAAA");//追加　機体名

}

// ============================================
// メインループ
// ============================================
void loop() {
  // 色を計測（頻度を下げる）
  static unsigned long lastColorRead = 0;
  if (millis() - lastColorRead > 100) {
    color_sensor.previous_color = color_sensor.current_color;
    float r, g, b;
    color_sensor.getRGB(r, g, b);
    color_sensor.current_color = color_sensor.identifyColor((int)r, (int)g, (int)b);
    lastColorRead = millis();
  }

  task();
  printStatus();
}