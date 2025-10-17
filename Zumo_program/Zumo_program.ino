#include "definitions.h"

// ============================================
// モーター速度定数
// ============================================
const int SPEED_ROTATE     = 140;
const int SPEED_FORWARD    = 140;
const int SPEED_ESCAPE     = 140;
const int SPEED_REVERSE    = -140;
const int SPEED_AVOID_ROT  = 140;
const int SPEED_MOVE       = 140;
const int SPEED_STOP       = 0;

// ============================================
// 目標方位角設定
// ============================================
const float TARGET_HEADING = 210.0;

// ============================================
// PI制御定数
// ============================================
const float KP = 2.0;
const float TIinv = 2.0 / 1000.0;

// ============================================
// グローバル変数定義
// ============================================
Pushbutton button(ZUMO_BUTTON);
ZumoMotors motors;
int motorL, motorR;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X);
float r, g, b;
unsigned int r_min = 60, g_min = 52, b_min = 62;
unsigned int r_max = 255, g_max = 255, b_max = 255;
int color, prevColor;

LSM303 compass;
float mx, my;
float heading_G;

const int trig = 2;
const int echo = 4;
int dist;

int mode;
int prevMode = -1;
unsigned long start_time;
unsigned long searchStartTime;
unsigned long timeNow_G, timePrev_G;
int searchRotationCount = 0;
bool objectDetectedInSearch = false;

float sum_e = 0;

// ============================================
// セットアップ
// ============================================
void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("========================================");
  Serial.println("  Zumo Robot Control System v2.1");
  Serial.println("========================================");
  Serial.print("Target Heading: ");
  Serial.print(TARGET_HEADING);
  Serial.println(" degrees");
  Serial.println("----------------------------------------");
  
  // カラーセンサー初期化
  if (tcs.begin()) {
    Serial.println("[INIT] Color sensor initialized");
  } else {
    Serial.println("[ERROR] Color sensor not found!");
    while (1);
  }
  
  // 地磁気センサー初期化
  compass.init();
  compass.enableDefault();
  compass.m_min.x = -32767;
  compass.m_min.y = -32767;
  compass.m_max.x = 32767;
  compass.m_max.y = 32767;
  Serial.println("[INIT] Compass initialized");
  
  // ピン設定
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  Serial.println("[INIT] Pins configured");
  
  // キャリブレーション実施
  Serial.println("----------------------------------------");
  Serial.println("[CALIB] Starting COMPASS CALIBRATION...");
  Serial.println("[CALIB] Press button to start");
  button.waitForButton();
  
  Serial.println("[CALIB] ROTATE the robot for 10 seconds");
  calibrationCompass();
  Serial.println("[CALIB] Compass calibration COMPLETE!");
  Serial.print("[CALIB] X range: ");
  Serial.print(compass.m_min.x);
  Serial.print(" to ");
  Serial.println(compass.m_max.x);
  Serial.print("[CALIB] Y range: ");
  Serial.print(compass.m_min.y);
  Serial.print(" to ");
  Serial.println(compass.m_max.y);
  Serial.println("----------------------------------------");
  
  // カラーセンサーキャリブレーション
  Serial.println("[CALIB] Press button to START COLOR CALIBRATION");
  button.waitForButton();
  calibrationColorSensor();
  Serial.println("----------------------------------------");

  // 動作開始待機
  Serial.println("[READY] Press button to START OPERATION");
  button.waitForButton();

  // 初期化完了
  mode = INIT;
  prevMode = -1;
  motorL = motorR = SPEED_STOP;
  timeNow_G = timePrev_G = millis();
  
  Serial.println("[READY] System ready! Starting operation...");
  Serial.println("========================================");
  Serial.println();
}

// ============================================
// メインループ
// ============================================
void loop() {
  // モーター駆動
  motors.setSpeeds(motorL, motorR);

  // 色を計測
  prevColor = color;
  getRGB(r, g, b);
  color = identify_color((int)r, (int)g, (int)b);

  // 距離計測
  dist = distance();

  // タスク実行
  task();
  
  // ステータス表示
  printStatus();
}