/*
 * Zumo_program.ino
 * 
 * 【機能概要】
 * Zumoロボットのメインプログラム
 * setup()とloop()を実装し、各モジュールを初期化・実行する
 * 
 * 【バージョン】
 * v3.1 - 坂道検知・登坂機能を追加
 * 
 * 【主要機能】
 * - setup(): センサーの初期化とキャリブレーション
 * - loop(): センサー読み取りとタスク実行
 */

#include "definitions.h"

// ============================================
// グローバルオブジェクト定義
// ============================================
// 各モジュールで使用するグローバルオブジェクトを実体化
Pushbutton button(ZUMO_BUTTON);         // ボタン（Zumoの前面ボタン）
MotorController motor_ctrl;             // モーター制御
ColorSensorState color_sensor;          // カラーセンサー
CompassState compass_state;             // 地磁気センサー・加速度センサー
UltrasonicSensor ultrasonic(2, 4);      // 超音波センサー（トリガー:2, エコー:4）
RobotState robot_state;                 // ロボットの状態
PIController pi_ctrl;                   // PI制御

// ============================================
// 定数定義
// ============================================
float TARGET_HEADING = 0.0;       // 目標方位角（度）
const float MAGNETIC_DECLINATION = -7.67;  // 磁気偏角（度、地域によって異なる）
const char ROBOT_NAME[] PROGMEM = "oka";  // ← ロボット名（必要に応じて変更）

// ============================================
// ボタン待機関数（簡略版）
// ============================================
/**
 * ボタンが押されるまで待機する関数
 * キャリブレーション開始前などに使用
 */
void waitForButtonPress() {
  Serial.println(F("Press button..."));
  
  // ボタンが押されるまでループ
  while (!button.isPressed()) {
    delay(50);  // 50ms間隔でチェック
  }
  
  // ボタンが離されるまで待機（チャタリング防止）
  while (button.isPressed()) {
    delay(10);
  }
  
  Serial.println(F("OK!"));
  delay(300);  // 安定化のための待機
}

// ============================================
// セットアップ
// ============================================
/**
 * 初期化処理
 * 各センサーを初期化し、キャリブレーションを実行
 * この関数は起動時に1回だけ実行される
 */
void setup() {
  // ========================================
  // シリアル通信の初期化
  // ========================================
  // 9600bpsで初期化（これより速いと認識しないかも）
  Serial.begin(9600);
  delay(1500);  // シリアル通信の安定化を待つ

  // バージョン情報を表示
  Serial.println(F("\n=== Zumo v3.2 ==="));
  
  // ========================================
  // カラーセンサーの初期化
  // ========================================
  Serial.print(F("Color sensor..."));
  if (color_sensor.tcs.begin()) {
    // 初期化成功
    Serial.println(F("OK"));
  } else {
    // 初期化失敗：停止
    Serial.println(F("FAIL"));
    while (1) delay(1000);  // 無限ループで停止
  }
  
  // ========================================
  // 地磁気センサーの初期化
  // ========================================
  Serial.print(F("Compass..."));
  compass_state.compass.init();           // センサーを初期化
  compass_state.compass.enableDefault();  // デフォルト設定を有効化（加速度計も有効）
  
  // 初期の最小値・最大値を設定（後でキャリブレーションで更新）
  compass_state.compass.m_min.x = -32767;
  compass_state.compass.m_min.y = -32767;
  compass_state.compass.m_max.x = 32767;
  compass_state.compass.m_max.y = 32767;
  Serial.println(F("OK"));
  
  // ========================================
  // 超音波センサーの初期化
  // ========================================
  Serial.print(F("Ultrasonic..."));
  ultrasonic.init();  // ピンを設定
  Serial.println(F("OK"));

    // ========================================
  // ゴール色のリクエストと受信
  // ========================================
  Serial.println(F("--- Requesting Goal Color ---"));
  Serial.println("REQUEST_COLOR");  // Processingにリクエスト送信

  unsigned long startTime = millis();
  while (!Serial.available()) {
    if (millis() - startTime > 3000) {
      Serial.println(F("Timeout: No color response"));
      break;
    }
    delay(10);
  }

/*
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'B') {
      TARGET_HEADING = 210.0 - 180;
      Serial.println("GOAL_COLOR:B");  //確認送信
    } else if (c == 'R') {
      TARGET_HEADING = 210.0 + 0.0;
      Serial.println("GOAL_COLOR:R");  //確認送信
    } else {
      Serial.print(F("Unexpected color code: "));
      Serial.println(c);
    }
  }
  */
  
  // ========================================
  // コンパスキャリブレーション
  // ========================================
  Serial.println(F("--- Compass Calib ---"));
  waitForButtonPress();  // ボタンが押されるまで待機
  calibrationCompassAdvanced();  // キャリブレーション実行（15秒間回転）
  Serial.println(F("Done!"));

  //キャリブレーション後の現在の方位をTRAGET_HEADINGに設定
  //ロボットを自分のゴールの方向に向ける
  Serial.println(F("--- Setting TARGET_HEADING ---"));
  waitForButtonPress();  // ボタンが押されるまで待機

  //ボタンが押された瞬間の向きを取得して設定
  compass_state.updateHeading(MAGNETIC_DECLINATION);
  for(int i=0; i<10; i++) {
    compass_state.updateHeading(MAGNETIC_DECLINATION);
    delay(10);
  }
  TARGET_HEADING = compass_state.current_heading;
  Serial.print(F("TARGET_HEADING set to: "));
  Serial.println(TARGET_HEADING, 1);

  // ========================================
  // カラーキャリブレーション
  // ========================================
  Serial.println(F("--- Color Calib ---"));
  waitForButtonPress();  // ボタンが押されるまで待機
  color_sensor.calibrate();  // キャリブレーション実行（2秒間前進）
  Serial.println(F("Done!"));

  // ========================================
  // 動作開始
  // ========================================
  Serial.println(F("--- Start ---"));
  waitForButtonPress();  // ボタンが押されるまで待機

  // ロボットの状態を初期化
  robot_state.mode = STATE_INIT;
  robot_state.time_now = robot_state.time_prev = millis();
  
  Serial.println(F("Running..."));

  // 機体名を送信（追加）
  Serial.println("NAME:oka");

}

// ============================================
// メインループ
// ============================================
/**
 * メインループ
 * センサーの読み取りとタスクの実行を繰り返す
 * この関数は永続的に実行される
 */
void loop() {
  // ========================================
  // 色を計測（頻度を下げる）
  // ========================================
  // 前回色を読み取った時刻を記録
  static unsigned long lastColorRead = 0;
  
  // 100ms経過していれば色を読み取る
  if (millis() - lastColorRead > 100) {
    // 前回の色を保存
    color_sensor.previous_color = color_sensor.current_color;
    
    // RGB値を取得
    float r, g, b;
    color_sensor.getRGB(r, g, b);
    
    // 色を識別
    color_sensor.current_color = color_sensor.identifyColor((int)r, (int)g, (int)b);
    
    // 最後に読み取った時刻を更新
    lastColorRead = millis();
  }

  // ========================================
  // デバッグ：加速度Z軸の値を表示
  // ========================================
  /*
  static unsigned long lastAccelDebug = 0;
  if (millis() - lastAccelDebug > 500) {
    compass_state.compass.readAcc();
    int accel_z = compass_state.compass.a.z + ACCEL_Z_OFFSET;
    Serial.print(F("ACCEL_Z:"));
    Serial.println(accel_z);
    lastAccelDebug = millis();
  }
  */

  // ========================================
  // タスクを実行（状態遷移）
  // ========================================
  task();  // state_machine.inoのtask()を呼び出し
  
  // ========================================
  // ステータスを表示
  // ========================================
  printStatus();  // state_machine.inoのprintStatus()を呼び出し
}