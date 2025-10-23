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

// ============================================
// 定数定義
// ============================================
const float TARGET_HEADING = 177.0;
const float MAGNETIC_DECLINATION = -7.0;

// ============================================
// セットアップ
// ============================================
void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("========================================");
  Serial.println("  Zumo Robot Control System v3.0");
  Serial.println("========================================");
  Serial.print("Target Heading: ");
  Serial.print(TARGET_HEADING);
  Serial.println(" degrees");
  Serial.println("----------------------------------------");
  
  // カラーセンサー初期化
  if (color_sensor.tcs.begin()) {
    Serial.println("[INIT] Color sensor initialized");
  } else {
    Serial.println("[ERROR] Color sensor not found!");
    while (1);
  }
  
  // 地磁気センサー初期化
  compass_state.compass.init();
  compass_state.compass.enableDefault();
  compass_state.compass.m_min.x = -32767;
  compass_state.compass.m_min.y = -32767;
  compass_state.compass.m_max.x = 32767;
  compass_state.compass.m_max.y = 32767;
  Serial.println("[INIT] Compass initialized");
  
  // 超音波センサー初期化
  ultrasonic.init();
  Serial.println("[INIT] Ultrasonic sensor initialized");
  
  // キャリブレーション実施
  Serial.println("----------------------------------------");
  Serial.println("[CALIB] Press button to start COMPASS calibration");
  button.waitForButton();
  
  Serial.println("[CALIB] ROTATE the robot for 15 seconds");
  calibrationCompassAdvanced();
  Serial.println("[CALIB] Compass calibration COMPLETE!");
  Serial.println("----------------------------------------");
  
  // カラーセンサーキャリブレーション
  Serial.println("[CALIB] Press button to START COLOR calibration");
  button.waitForButton();
  color_sensor.calibrate();
  Serial.println("----------------------------------------");

  // 動作開始待機
  Serial.println("[READY] Press button to START OPERATION");
  button.waitForButton();

  // 初期化完了
  robot_state.mode = STATE_INIT;
  robot_state.time_now = robot_state.time_prev = millis();
  
  Serial.println("[READY] System ready! Starting operation...");
  Serial.println("========================================");
  Serial.println();
}

// ============================================
// メインループ
// ============================================
void loop() {
  // 色を計測
  color_sensor.previous_color = color_sensor.current_color;
  float r, g, b;
  color_sensor.getRGB(r, g, b);
  color_sensor.current_color = color_sensor.identifyColor((int)r, (int)g, (int)b);

  // タスク実行
  task();
  
  // ステータス表示
  printStatus();
}