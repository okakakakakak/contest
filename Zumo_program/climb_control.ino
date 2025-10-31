#include "definitions.h"

// ============================================
// 加速度センサーによる傾斜検知（Pitch角計算版）
// ============================================
/**
 * 坂道（傾斜）を検知する関数
 * 加速度センサーの値からPitch角を計算し、閾値と比較して傾斜を判定する
 * 連続して閾値を超えた場合のみ坂道と判定（誤検知防止）
 * 
 * @return true: 坂道を検知した / false: 平坦な地面
 */
bool isSlopeDetected() {
  static unsigned long lastAccelRead = 0;
  static float current_pitch = 0.0;
  static int slope_detect_count = 0;  // 連続検知カウンター
  
  // 頻繁に読み取ると動作に影響するため、一定間隔(ACCEL_READ_INTERVAL)で読み取る
  if (millis() - lastAccelRead > ACCEL_READ_INTERVAL) {
    // 加速度センサーから最新の値を読み取る
    compass_state.compass.readAcc();
    
    // 各軸の加速度を取得（生の値）
    float a_x = compass_state.compass.a.x;
    float a_y = compass_state.compass.a.y;
    float a_z = compass_state.compass.a.z;
    
    // 加速度ベクトルのノルム（大きさ）を計算
    float norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    
    // ゼロ除算を防ぐ
    if (norm < 100) {
      norm = 100;  // 最小値を設定
    }
    
    // 加速度を正規化（重力加速度 = 1 の単位系にする）
    float a_x_normalized = a_x / norm;
    
    // Pitch角を計算（ラジアン）
    // Pitch = arcsin(-a_x_normalized)
    // -1 ≤ a_x_normalized ≤ 1 の範囲に制限
    a_x_normalized = constrain(a_x_normalized, -1.0, 1.0);
    float pitch_rad = asin(-a_x_normalized);
    
    // 度に変換
    current_pitch = pitch_rad * 180.0 / PI;
    
    // 最後に読み取った時刻を更新
    lastAccelRead = millis();
    
    // デバッグ出力（500msごと）
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
      Serial.print(F("ACCEL_X:"));
      Serial.print((int)a_x);
      Serial.print(F(" ACCEL_Y:"));
      Serial.print((int)a_y);
      Serial.print(F(" ACCEL_Z:"));
      Serial.print((int)a_z);
      Serial.print(F(" PITCH:"));
      Serial.println(current_pitch, 1);  // 小数点1桁
      lastDebug = millis();
    }
    
    // 閾値判定（絶対値で判定 - 上り坂も下り坂も検知）
    if (abs(current_pitch) > SLOPE_PITCH_THRESHOLD) {
      slope_detect_count++;  // 検知カウントを増やす
    } else {
      slope_detect_count = 0;  // 閾値未満ならリセット
    }
  }
  
  // 連続して3回以上検知した場合のみ坂道と判定
  if (slope_detect_count >= 3) {
    return true;  // 坂道を検知
  }
  return false;  // 平坦な地面
}

// ============================================
// 登頂判定（傾斜がなくなったかチェック）
// ============================================
/**
 * 坂道を登りきった（平地に戻った）かを判定する関数
 * 坂道モード中のみ使用
 * 
 * @return true: 平地に戻った / false: まだ坂道
 */
bool hasReachedTop() {
  static unsigned long lastAccelRead = 0;
  static float current_pitch = 0.0;
  static int flat_detect_count = 0;  // 平地連続検知カウンター
  
  // 一定間隔で読み取る
  if (millis() - lastAccelRead > ACCEL_READ_INTERVAL) {
    // 加速度センサーから最新の値を読み取る
    compass_state.compass.readAcc();
    
    // 各軸の加速度を取得
    float a_x = compass_state.compass.a.x;
    float a_y = compass_state.compass.a.y;
    float a_z = compass_state.compass.a.z;
    
    // ノルムを計算
    float norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    if (norm < 100) {
      norm = 100;
    }
    
    // 正規化
    float a_x_normalized = a_x / norm;
    a_x_normalized = constrain(a_x_normalized, -1.0, 1.0);
    
    // Pitch角を計算
    float pitch_rad = asin(-a_x_normalized);
    current_pitch = pitch_rad * 180.0 / PI;
    
    lastAccelRead = millis();
    
    // デバッグ出力
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
      Serial.print(F("CLIMBING - PITCH:"));
      Serial.println(current_pitch, 1);
      lastDebug = millis();
    }
    
    // 平地判定（閾値の半分以下で平地とみなす）
    if (abs(current_pitch) < SLOPE_PITCH_THRESHOLD / 2.0) {
      flat_detect_count++;
    } else {
      flat_detect_count = 0;
    }
  }
  
  // 連続して3回以上平地を検知したら登頂完了
  if (flat_detect_count >= 3) {
    flat_detect_count = 0;  // カウンターをリセット
    return true;
  }
  return false;
}

// ============================================
// 坂道登坂モードの実行
// ============================================
/**
 * 坂道を登るモードの制御ロジック
 * PI制御で目標方位を維持しながら、高速で前進する
 * 終了条件：黒線検知、または傾斜がなくなる（登頂）
 */
void runClimbMode() {
  // 超音波センサーで前方の距離を計測
  int dist = ultrasonic.getDistance();
  
  // ========================================
  // 終了条件1: 黒線（坂の頂上やライン）を検出したら停止
  // ========================================
  if (color_sensor.current_color == COLOR_BLACK) {
    motor_ctrl.stop();  // モーターを停止
    
    // STATE_STOPではなくAVOIDモードに遷移（黒線を回避する）
    robot_state.mode = STATE_AVOID;
    robot_state.state_start_time = millis();  // 状態開始時刻を記録
    return;  // 関数を終了
  }
  
  // ========================================
  // 💡 終了条件2: 傾斜がなくなった（山を登頂した）場合
  // ========================================
  if (hasReachedTop()) {  // isSlopeDetected() → hasReachedTop() に変更
    motor_ctrl.stop();  // モーターを停止
    
    Serial.println(F("Reached top!"));
    
    // 宝の検知条件：距離が30cm未満
    if (dist > 0 && dist < 30) { 
      // 登頂成功後、宝を検知 → STATE_CHECK_STATIC を経由して STATE_APPROACH へ
      // 静止物体かどうかを確認してから接近する
      robot_state.mode = STATE_CHECK_STATIC;
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
    } else {
      // 登頂成功したが宝は検知せず → STATE_SEARCH へ戻る
      // 探索モードで宝を探す
      robot_state.mode = STATE_SEARCH;
      robot_state.search_start_time = millis();
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
    }
    
    // PI制御の積分項をリセット
    pi_ctrl.reset();
    return;  // 関数を終了
  }

  // ========================================
  // 登坂継続：目標方位(TARGET_HEADING)に向かってPI制御を行う
  // ========================================
  // PI制御による方位維持は motion_control.ino の turnTo() 関数を使用
  // 制御入力 u が返される（正：左旋回、負：右旋回）
  float control_u = turnTo(TARGET_HEADING);
  
  // 制御入力が小さい場合は中央値を0にする（モーターの遊び対策）
  // 微小な制御入力では効果がないため、閾値以下は無視
  if (abs(control_u) < 5) {
    control_u = 0;
  }
  
  // 坂を登るための高い基本速度を設定
  const int CLIMB_BASE_SPEED = 180;
  
  // 左右のスピードを計算（基本速度 + 制御入力）
  // control_u * 0.5 で制御の影響を調整（大きすぎると不安定になる）
  int left = CLIMB_BASE_SPEED + control_u * 0.5;   // 左モーター速度
  int right = CLIMB_BASE_SPEED - control_u * 0.5;  // 右モーター速度
  
  // スピードを制限（オーバーフロー・モーター保護）
  // -255〜255の範囲に制限
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  
  // モーターに速度を設定
  motor_ctrl.setSpeeds(left, right);
}

// ============================================
// 加速度センサーキャリブレーション（使用しない - 削除可能）
// ============================================
/**
 * この関数は使用しません。
 * Pitch角計算では生の加速度値を使用するため、
 * オフセットキャリブレーションは不要です。
 */
void calibrateAccelZOffset() {
  // 何もしない（削除してもOK）
  Serial.println(F("Accel calibration skipped (not needed for pitch calculation)"));
}