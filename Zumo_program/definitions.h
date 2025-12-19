/*
 * definitions.h
 * 
 * 【機能概要】
 * プログラム全体で使用する定数、構造体、グローバル変数の宣言を行うヘッダーファイル
 * 
 * 【主要な定義】
 * - 色の定義（白、黒、赤、青など）
 * - 状態の定義（探索、接近、回避など）
 * - モーター速度定数
 * - センサー関連の構造体
 * - ロボット状態を管理する構造体
 */

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// ============================================
// 必要なライブラリのインクルード
// ============================================
#include <ZumoMotors.h>          // Zumoロボットのモーター制御
#include <Adafruit_TCS34725.h>   // カラーセンサー（RGB値取得）
#include <LSM303.h>               // 地磁気センサー・加速度センサー
#include <Pushbutton.h>           // ボタン入力

// ============================================
// 色定義
// ============================================
// カラーセンサーで検知する色の識別コード
#define COLOR_WHITE    0  // 白色（通常のフィールド）
#define COLOR_BLACK    1  // 黒色（ライン、回避が必要）
#define COLOR_RED      2  // 赤色（自陣ゾーン）
#define COLOR_BLUE     3  // 青色（自陣ゾーン）
#define COLOR_OTHER    4  // その他の色（未分類）

// ============================================
// 状態定義
// ============================================
// ロボットの動作モードを表す状態コード
#define STATE_INIT              0   // 初期化状態
#define STATE_DIRECTION         1   // 💡 NEW: 目標方向を向く（一度きり）
#define STATE_SEARCH            2   // 探索状態（回転しながら物体を探す）
#define STATE_CHECK_STATIC      3   // 静止物体確認状態
#define STATE_APPROACH          4   // 接近状態（物体に近づく）
#define STATE_TURN_TO_TARGET    5   // 目標方位へ旋回状態
#define STATE_WAIT_AFTER_TURN   6   // 旋回後の待機状態
#define STATE_ESCAPE            7   // 脱出状態（物体を運搬中）
#define STATE_AVOID             8   // 回避状態（黒線を避ける）
#define STATE_STOP              9   // 停止状態
#define STATE_MOVE              10   // 移動状態（前進）
#define STATE_CLIMB             11  // 坂道登坂モード
#define STATE_CHECK_ZONE        12  // ゾーン確認状態（STATE_CLIMBの挿入で1つずれる）
#define STATE_DEPOSIT           13  // 預け入れ状態（STATE_CLIMBの挿入で1つずれる）
#define STATE_STACK             14  // スタック検知モード


// ============================================
// モーター速度定数
// ============================================
// 各状態で使用するモーター速度の基準値
#define MOTOR_ROTATE     100   // 回転時の速度（140 → 210）
#define MOTOR_FORWARD    210   // 前進時の速度（140 → 210）
#define MOTOR_ESCAPE     210   // 脱出時の速度（140 → 210）
#define MOTOR_REVERSE    -210  // 後退時の速度（-140 → -210）
#define MOTOR_AVOID_ROT  140   // 回避時の回転速度（140 → 210）
#define MOTOR_MOVE       210   // 移動時の速度（140 → 210）
#define MOTOR_STOP       0     // 停止（速度0）
#define MOTOR_TURN       180   // 旋回速度の基本値（120 → 180、弧を描くための前進成分）

// ============================================
// 加速度センサー定数
// ============================================
// 坂道検知に使用する加速度センサーのパラメータ
#define ACCEL_READ_INTERVAL     50   // 加速度センサーの計測間隔 (ms)
#define SLOPE_PITCH_THRESHOLD   15.0    // 傾斜判定の閾値（Pitch角、度）

// ============================================
// PI制御パラメータ
// ============================================
/**
 * PI制御（比例積分制御）のパラメータを管理する構造体
 * 方位制御に使用し、目標方位に向かって安定した旋回を実現する
 */
struct PIController {
  float kp;       // 比例ゲイン（誤差に対する応答の強さ）
  float ti_inv;   // 積分時間の逆数（積分項の影響度）
  float sum_e;    // 誤差の積分値（累積誤差）
  
  // コンストラクタ：デフォルト値を設定
  PIController() : kp(4.0), ti_inv(0.004), sum_e(0) {}
  
  // 積分項をリセット（状態遷移時などに使用）
  void reset() { sum_e = 0; }
};

// ============================================
// 地磁気センサー補正用構造体（簡略化）
// ============================================
/**
 * 地磁気センサーのキャリブレーション値を保持する構造体
 * ハードアイアン補正とスケール補正を行う
 */
struct MagnetometerCalibration {
  float offset_x, offset_y;  // オフセット（センサーの中心位置のズレ）
  float scale_x, scale_y;    // スケール（X軸とY軸の感度の違い）
  
  // コンストラクタ：デフォルト値（補正なし）
  MagnetometerCalibration() : offset_x(0), offset_y(0), scale_x(1.0), scale_y(1.0) {}
};

// ============================================
// コンパス状態構造体（バッファサイズ削減）
// ============================================
#define HEADING_FILTER_SIZE 3  // 方位角フィルタのバッファサイズ（5→3に削減）

/**
 * 地磁気センサーの状態を管理する構造体
 * 方位角の計算と移動平均フィルタを実装
 */
struct CompassState {
  LSM303 compass;                              // LSM303センサーオブジェクト
  MagnetometerCalibration calib;               // キャリブレーション値
  float heading_buffer[HEADING_FILTER_SIZE];   // 方位角の履歴（移動平均用）
  byte heading_index;                          // バッファの現在位置（int→byteに変更）
  float current_heading;                       // 現在の方位角（0〜360度）
  
  // コンストラクタ：初期化
  CompassState() : heading_index(0), current_heading(0) {
    // 方位角バッファを0で初期化
    for (byte i = 0; i < HEADING_FILTER_SIZE; i++) {
      heading_buffer[i] = 0;
    }
  }
  
  // 方位角を更新する関数（sensors.inoで実装）
  void updateHeading(float magnetic_declination);
};

// ============================================
// カラーセンサー状態構造体
// ============================================
/**
 * カラーセンサーの状態を管理する構造体
 * RGB値の取得とキャリブレーション、色の識別を行う
 */
struct ColorSensorState {
  Adafruit_TCS34725 tcs;  // TCS34725カラーセンサーオブジェクト
  
  // キャリブレーション値（最小値と最大値）
  uint16_t r_min, g_min, b_min;  // RGB各成分の最小値（unsigned int→uint16_tに明示）
  uint16_t r_max, g_max, b_max;  // RGB各成分の最大値
  
  byte current_color;   // 現在検知している色（int→byteに変更）
  byte previous_color;  // 前回検知した色（int→byteに変更）
  
  // コンストラクタ：センサーの設定とデフォルト値
  ColorSensorState() : 
    tcs(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X),  // センサーの積分時間とゲイン
    r_min(60), g_min(52), b_min(62),      // 初期最小値
    r_max(255), g_max(255), b_max(255),   // 初期最大値
    current_color(COLOR_OTHER), previous_color(COLOR_OTHER) {}
  
  // RGB値を取得する関数（sensors.inoで実装）
  void getRGB(float& r, float& g, float& b);
  
  // キャリブレーションを行う関数（sensors.inoで実装）
  void calibrate();
  
  // RGB値から色を識別する関数（sensors.inoで実装）
  byte identifyColor(int r, int g, int b);
};

// ============================================
// 距離センサー構造体
// ============================================
/**
 * 超音波センサーを管理する構造体
 * 距離の計測と物体の静止判定を行う
 */
struct UltrasonicSensor {
  byte trig_pin;  // トリガーピン番号（int→byteに変更）
  byte echo_pin;  // エコーピン番号（int→byteに変更）
  
  // コンストラクタ：ピン番号を設定
  UltrasonicSensor(byte trig, byte echo) : trig_pin(trig), echo_pin(echo) {}
  
  // センサーを初期化する関数（sensors.inoで実装）
  void init();
  
  // 距離を計測する関数（sensors.inoで実装）
  int getDistance();
  
  // 物体が静止しているか判定する関数（sensors.inoで実装）
  bool isObjectStatic();
};

// ============================================
// ロボット状態構造体（登坂フェーズ追加版）
// ============================================
/**
 * ロボット全体の状態を管理する構造体
 * 現在のモード、時刻、探索状態などを保持
 */
struct RobotState {
  byte mode;           // 現在の動作モード（int→byteに変更）
  byte previous_mode;  // 前回の動作モード（int→byteに変更）
  
  unsigned long state_start_time;   // 現在の状態に入った時刻
  unsigned long search_start_time;  // 探索を開始した時刻
  
  byte search_rotation_count;       // 探索中の回転カウント（int→byteに変更）
  bool object_detected_in_search;   // 探索中に物体を検知したか
  
  unsigned long time_now;   // 現在時刻
  unsigned long time_prev;  // 前回の時刻
  
  byte cups_delivered;  // 運搬したカップの数

  // ★ スタック判定用フラグ
  bool allow_stack_check;
  
  // 💡 NEW: 登坂モード用の変数
  float climb_start_heading;   // 登坂開始時の方位角（使用しない - 予約）
  byte climb_phase;            // 登坂のフェーズ（0:後退、1:左旋回、2:大回り、3:右旋回、4:前進、5:登坂）
  
  // コンストラクタ：初期化
  RobotState() : 
    mode(STATE_INIT), previous_mode(255),  // -1の代わりに255（byteの最大値）
    state_start_time(0), search_start_time(0),
    search_rotation_count(0), object_detected_in_search(false),
    time_now(0), time_prev(0), cups_delivered(0),
    climb_start_heading(0), climb_phase(0) {}
  
  // 時刻を更新する関数
  void updateTime() {
    time_prev = time_now;
    time_now = millis();
  }
};

// ============================================
// モーター制御構造体
// ============================================
/**
 * モーターの制御を管理する構造体
 * 左右のモーター速度を設定・保持する
 */
struct MotorController {
  ZumoMotors motors;  // ZumoMotorsオブジェクト
  
  int left_speed;   // 左モーターの速度
  int right_speed;  // 右モーターの速度
  
  // コンストラクタ：速度を0で初期化
  MotorController() : left_speed(0), right_speed(0) {}
  
  // モーター速度を設定する関数
  void setSpeeds(int left, int right) {
    left_speed = left;
    right_speed = right;
    motors.setSpeeds(left_speed, right_speed);
  }
  
  // モーターを停止する関数
  void stop() {
    setSpeeds(0, 0);
  }
};

// ============================================
// スタック検知用パラメータ（回転ベース版）
// ============================================

#define STACK_CHECK_INTERVAL      200   // 方位チェックの間隔 (ms)

// 1. 直進時の異常回転検知用（新規追加）
// 直進中なのに、200msでこれ以上回っていたら「絡まって回されている」とみなす
#define MAX_STRAIGHT_ERROR_ANGLE  15.0  // 度 (直進中の許容ブレ幅)

// 2. 回転時の回転不足検知用
// 回転中なのに、200msでこれ以下しか回っていなかったら「引っかかっている」とみなす
#define MIN_TURN_ANGLE            3.0   // 度 (少し厳しめに3.0度推奨)

// ※ IMPACT_THRESHOLD は不要になったため削除しました

// ============================================
// グローバルオブジェクト（必要最小限）
// ============================================
// 他のファイルから参照できるようにexternで宣言
extern Pushbutton button;                // ボタン
extern MotorController motor_ctrl;       // モーター制御
extern ColorSensorState color_sensor;    // カラーセンサー
extern CompassState compass_state;       // 地磁気センサー
extern UltrasonicSensor ultrasonic;      // 超音波センサー
extern RobotState robot_state;           // ロボット状態
extern PIController pi_ctrl;             // PI制御

// ============================================
// 定数（PROGMEM使用）
// ============================================
// プログラムメモリに格納される定数
extern float TARGET_HEADING;        // 目標方位角（度）
extern const float MAGNETIC_DECLINATION;  // 磁気偏角（度）

// ============================================
// 関数プロトタイプ
// ============================================
// 他のファイルで実装される関数の宣言
void printModeChange();                   // モード変更を表示
void printStatus();                       // ステータスを表示
void task();                              // メインタスク（状態遷移）
float turnTo(float target_heading);       // 目標方位に旋回（PI制御）
void calibrationCompassAdvanced();        // コンパスキャリブレーション
bool isSlopeDetected();                   // 傾斜検知
bool hasReachedTop();                     // 登頂判定
void runClimbMode();                      // 登坂モード実行
void calibrateAccelZOffset();             // Z軸オフセットキャリブレーション

#endif