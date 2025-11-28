/*
 * sensors.ino
 * 
 * 【機能概要】
 * 各種センサーの読み取りと処理を行うモジュール
 * 
 * 【主要機能】
 * 1. ColorSensorState: カラーセンサーのRGB取得、キャリブレーション、色識別
 * 2. UltrasonicSensor: 超音波センサーの距離計測、静止判定
 * 3. CompassState: 地磁気センサーの方位角計算
 * 4. calibrationCompassAdvanced(): コンパスのキャリブレーション
 */

#include "definitions.h"

// ============================================
// ColorSensorState メソッド実装
// ============================================

/**
 * RGB値を取得する関数
 * センサーから生の値を読み取り、キャリブレーション値で0〜255にマッピング
 * 
 * @param r 赤成分（出力、0〜255）
 * @param g 緑成分（出力、0〜255）
 * @param b 青成分（出力、0〜255）
 */
void ColorSensorState::getRGB(float& r, float& g, float& b) {
  // センサーから生のRGB値とクリア値を取得
  uint16_t r_raw, g_raw, b_raw, clr;
  tcs.getRawData(&r_raw, &g_raw, &b_raw, &clr);
 
  // 生の値をキャリブレーション値（min/max）を使って0〜255にマッピング
  // map(値, 入力最小, 入力最大, 出力最小, 出力最大)
  r = map(r_raw, r_min, r_max, 0, 255);
  g = map(g_raw, g_min, g_max, 0, 255);
  b = map(b_raw, b_min, b_max, 0, 255);

  // 値を0〜255の範囲に制限（オーバーフロー防止）
  r = constrain(r, 0.0, 255.0);
  g = constrain(g, 0.0, 255.0);
  b = constrain(b, 0.0, 255.0);
}

/**
 * RGB値から色を識別する関数
 * 閾値を使って白、黒、赤、青、その他を判定
 * 
 * @param r 赤成分（0〜255）
 * @param g 緑成分（0〜255）
 * @param b 青成分（0〜255）
 * @return 色コード（COLOR_WHITE, COLOR_BLACK, COLOR_RED, COLOR_BLUE, COLOR_OTHER）
 */
byte ColorSensorState::identifyColor(int r, int g, int b) {
  // 白（全ての値が高い）
  // RGB全てが240以上なら白と判定
  if (r > 240 && g > 240 && b > 240) return COLOR_WHITE;
  
  // 黒（全ての値が低い）
  // RGB全てが10未満なら黒と判定
  if (r < 10 && g < 10 && b < 10) return COLOR_BLACK;
  
  // 赤（Rが高く、G・Bが低い）
  // 赤成分が100以上、緑と青が80未満なら赤と判定
  if (r > 100 && g < 80 && b < 80) return COLOR_RED;
  
  // 青（Bが高く、R・Gが低い）
  // 青成分が100以上、赤と緑が80未満なら青と判定
  if (r < 80 && g < 80 && b > 100) return COLOR_BLUE;
  
  // その他（上記に当てはまらない色）
  return COLOR_OTHER;
}

/**
 * カラーセンサーのキャリブレーションを行う関数
 * ロボットを回転させながら、RGB各成分の最小値と最大値を記録
 * これにより照明条件が変わっても安定した色検知が可能
 */
void ColorSensorState::calibrate() {
  Serial.println(F("Calibrating..."));
  
  // キャリブレーション中は低速で前進
  motor_ctrl.setSpeeds(90, 90);

  // 最小値を最大値で初期化（後で小さい値で上書き）
  r_min = 65535;
  g_min = 65535;
  b_min = 65535;
  
  // 最大値を0で初期化（後で大きい値で上書き）
  r_max = 0;
  g_max = 0;
  b_max = 0;

  // キャリブレーション開始時刻を記録
  unsigned long start_time = millis();

  // 2秒間センサーから値を読み取り続ける
  while (millis() - start_time < 2000) {
    // RGB値を取得
    uint16_t r, g, b, clr;
    tcs.getRawData(&r, &g, &b, &clr);
  
    // 各成分の最小値を更新
    if (r < r_min) r_min = r;
    if (g < g_min) g_min = g;
    if (b < b_min) b_min = b;
    
    // 各成分の最大値を更新
    if (r > r_max) r_max = r;
    if (g > g_max) g_max = g;
    if (b > b_max) b_max = b;
    
    delay(50);  // 50ms間隔で読み取り
  }
  
  // キャリブレーション完了後、停止
  motor_ctrl.stop();
  
  // 値が有効かチェック（簡略版）
  // もし最大値が最小値以下なら、デフォルト値を使用
  if (r_max <= r_min) {
    r_min = 60;
    r_max = 255;
  }
  if (g_max <= g_min) {
    g_min = 52;
    g_max = 255;
  }
  if (b_max <= b_min) {
    b_min = 62;
    b_max = 255;
  }
}

// ============================================
// UltrasonicSensor メソッド実装
// ============================================

/**
 * 超音波センサーを初期化する関数
 * トリガーピンを出力、エコーピンを入力に設定
 */
void UltrasonicSensor::init() {
  pinMode(trig_pin, OUTPUT);  // トリガーピンを出力モードに
  pinMode(echo_pin, INPUT);   // エコーピンを入力モードに
}

/**
 * 超音波センサーで距離を計測する関数
 * 超音波を発射し、反射波が戻ってくるまでの時間から距離を計算
 * 
 * @return 距離（cm）、計測失敗時は100を返す
 */
int UltrasonicSensor::getDistance() {
  // トリガーピンをHIGHにして超音波を発射
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);  // 10マイクロ秒間HIGHを維持
  digitalWrite(trig_pin, LOW);
  
  // エコーピンがHIGHになる時間を計測（タイムアウト5767μs）
  // これは約1m（往復2m）に相当する時間
  unsigned long interval = pulseIn(echo_pin, HIGH, 5767);
  
  // 距離を計算
  // 音速 = 331.5 + 0.61 * 温度（m/s）、ここでは温度25度と仮定
  // 距離 = 音速 * 時間 / 2（往復なので2で割る）
  int dst = (0.61 * 25 + 331.5) * interval / 10000 / 2;
  
  // 計測失敗（dst==0）の場合は100cmを返す
  if (dst == 0) dst = 100;
  
  delay(60);  // 次の計測まで60ms待機（センサーの安定化）
  
  return dst;
}

/**
 * 物体が静止しているか判定する関数
 * 500ms間に複数回距離を計測し、変化が少なければ静止と判定
 * 
 * @return true: 静止している / false: 動いている
 */
bool UltrasonicSensor::isObjectStatic() {
  // 判定パラメータ
  const int checkDuration = 500;  // 判定時間（ms）
  const int interval = 100;       // 計測間隔（ms）
  const int threshold = 3;        // 変化回数の閾値
  const int delta = 2;            // 変化と見なす距離差（cm）

  // 判定中はロボットを停止
  motor_ctrl.stop();

  // 初回の距離を計測
  int prev = getDistance();
  
  int changes = 0;  // 距離が変化した回数
  unsigned long checkStart = millis();  // 判定開始時刻

  // checkDuration（500ms）間、繰り返し計測
  while (millis() - checkStart < checkDuration) {
    // 現在の距離を計測
    int now = getDistance();
    
    // 前回との距離差がdelta（2cm）を超えていれば変化とカウント
    if (abs(now - prev) > delta) {
      changes++;
    }
    
    // 現在の距離を保存
    prev = now;
    delay(interval);  // interval（100ms）待機
  }

  // 変化回数がthreshold（3回）未満なら静止と判定
  return (changes < threshold);
}

// ============================================
// CompassState メソッド実装
// ============================================

/**
 * 地磁気センサーから方位角を更新する関数
 * ハードアイアン補正、スケール補正、偏角補正、移動平均フィルタを適用
 * 
 * @param magnetic_declination 磁気偏角（度）
 */
void CompassState::updateHeading(float magnetic_declination) {
  // ========================================
  // ステップ1: センサーから地磁気データを読み取る
  // ========================================
  compass.read();
  
  // ========================================
  // ステップ2: ハードアイアン補正を適用
  // ========================================
  // ハードアイアン補正：センサーの中心位置のズレを補正
  // スケール補正：X軸とY軸の感度の違いを補正
  float mx_corrected = (compass.m.x - calib.offset_x) * calib.scale_x;
  float my_corrected = (compass.m.y - calib.offset_y) * calib.scale_y;
  
  // ========================================
  // ステップ3: 値を-128〜127にマッピング
  // ========================================
  // キャリブレーション時に記録したmin/maxを使用
  float mx = map(mx_corrected, 
                 compass.m_min.x - calib.offset_x, 
                 compass.m_max.x - calib.offset_x, 
                 -128, 127);
  float my = map(my_corrected,
                 compass.m_min.y - calib.offset_y,
                 compass.m_max.y - calib.offset_y,
                 -128, 127);
  
  // ========================================
  // ステップ4: 生の方位角を計算
  // ========================================
  // atan2(y, x)でX-Y平面上の角度を計算（ラジアン）
  // 180/πを掛けて度に変換
  float raw_heading = atan2(my, mx) * 180.0 / M_PI;
  
  // 角度を0〜360度の範囲に正規化
  if (raw_heading < 0) raw_heading += 360;
  
  // ========================================
  // ステップ5: 偏角補正
  // ========================================
  // 磁北と真北の差（磁気偏角）を補正
  raw_heading += magnetic_declination;
  
  // 再度0〜360度の範囲に正規化
  if (raw_heading < 0) raw_heading += 360;
  if (raw_heading >= 360) raw_heading -= 360;
  
  // ========================================
  // ステップ6: 移動平均フィルタ
  // ========================================
  // バッファに最新の方位角を保存
  heading_buffer[heading_index] = raw_heading;
  
  // インデックスを進める（循環バッファ）
  heading_index = (heading_index + 1) % HEADING_FILTER_SIZE;
  
  // バッファ内の全ての値の平均を計算
  float sum = 0;
  for (byte i = 0; i < HEADING_FILTER_SIZE; i++) {
    sum += heading_buffer[i];
  }
  current_heading = sum / HEADING_FILTER_SIZE;
}

// ============================================
// コンパスキャリブレーション（簡略版）
// ============================================

/**
 * 地磁気センサーの最小値・最大値を更新する補助関数
 * キャリブレーション中に呼び出される
 * 
 * @param x X軸の値
 * @param y Y軸の値
 * @param mx_min X軸の最小値（参照）
 * @param mx_max X軸の最大値（参照）
 * @param my_min Y軸の最小値（参照）
 * @param my_max Y軸の最大値（参照）
 */
static void updateMinMax(int x, int y, float& mx_min, float& mx_max, float& my_min, float& my_max) {
  // X軸の最小値・最大値を更新
  if (x < mx_min) mx_min = x;
  if (x > mx_max) mx_max = x;
  
  // Y軸の最小値・最大値を更新
  if (y < my_min) my_min = y;
  if (y > my_max) my_max = y;
}

/**
 * 地磁気センサーのキャリブレーションを行う関数
 * ロボットを回転させながら地磁気の最小値・最大値を記録し、
 * ハードアイアン補正とスケール補正の値を計算する
 */
void calibrationCompassAdvanced() {
  // 最小値・最大値を初期化
  float mx_max = -32768, my_max = -32768;  // 最大値を最小値で初期化
  float mx_min = 32767, my_min = 32767;    // 最小値を最大値で初期化
  
  Serial.println(F("Rotating 10s..."));
  
  unsigned long startTime = millis();
  
  // ========================================
  // フェーズ1: 時計回りに3.3秒間回転
  // ========================================
  while (millis() - startTime < 3300) {
    // 地磁気を読み取る
    compass_state.compass.read();
    
    // 最小値・最大値を更新
    updateMinMax(compass_state.compass.m.x, compass_state.compass.m.y, 
                 mx_min, mx_max, my_min, my_max);
    
    // 時計回りに回転（左モーター正転、右モーター逆転）
    motor_ctrl.setSpeeds(180, -180);
    delay(20);
  }
  
  // ========================================
  // フェーズ2: 反時計回りに3.3秒間回転
  // ========================================
  startTime = millis();
  while (millis() - startTime < 3300) {
    compass_state.compass.read();
    updateMinMax(compass_state.compass.m.x, compass_state.compass.m.y,
                 mx_min, mx_max, my_min, my_max);
    
    // 反時計回りに回転（左モーター逆転、右モーター正転）
    motor_ctrl.setSpeeds(-180, 180);
    delay(20);
  }
  
  // ========================================
  // フェーズ3: 再度時計回りに3.3秒間回転
  // ========================================
  startTime = millis();
  while (millis() - startTime < 3300) {
    compass_state.compass.read();
    updateMinMax(compass_state.compass.m.x, compass_state.compass.m.y,
                 mx_min, mx_max, my_min, my_max);
    motor_ctrl.setSpeeds(180, -180);
    delay(20);
  }
  
  // キャリブレーション完了後、停止
  motor_ctrl.stop();
  delay(500);
  
  // ========================================
  // ステップ1: ハードアイアン補正を計算
  // ========================================
  // オフセット = (最大値 + 最小値) / 2
  // これが地磁気センサーの中心位置
  compass_state.calib.offset_x = (mx_max + mx_min) / 2.0;
  compass_state.calib.offset_y = (my_max + my_min) / 2.0;
  
  // ========================================
  // ステップ2: スケール補正を計算
  // ========================================
  // 各軸の範囲 = (最大値 - 最小値) / 2
  float range_x = (mx_max - mx_min) / 2.0;
  float range_y = (my_max - my_min) / 2.0;
  
  // 平均範囲を計算
  float avg_range = (range_x + range_y) / 2.0;
  
  // スケール = 平均範囲 / 各軸の範囲
  // これでX軸とY軸の感度を揃える
  compass_state.calib.scale_x = avg_range / range_x;
  compass_state.calib.scale_y = avg_range / range_y;
  
  // ========================================
  // ステップ3: 最小値・最大値を保存
  // ========================================
  // compass.m_min/maxに保存（後で使用）
  compass_state.compass.m_min.x = mx_min;
  compass_state.compass.m_max.x = mx_max;
  compass_state.compass.m_min.y = my_min;
  compass_state.compass.m_max.y = my_max;
}