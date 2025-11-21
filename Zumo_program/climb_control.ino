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
// 坂道登坂モードの実行（後退→大回り→登坂）
// ============================================
/**
 * フェーズ0: 0.5秒後退
 * フェーズ1: 3秒間平地で大きく左旋回（山の周りを回る）
 * フェーズ2: 山の方向を向く
 * フェーズ3: 直進登坂（ロール角制御、カップ検知あり）
 */
void runClimbMode() {
  static unsigned long lastAccelRead = 0;
  static float current_roll = 0.0;
  static float roll_integral = 0.0;  // ロール角の積分項
  static unsigned long phase_start_time = 0;  // 各フェーズの開始時刻
  
  // 超音波センサーで前方の距離を計測
  int dist = ultrasonic.getDistance();
  
  // ========================================
  // 終了条件1: 黒線（坂の頂上やライン）を検出したら停止
  // ========================================
  if (color_sensor.current_color == COLOR_BLACK) {
    motor_ctrl.stop();
    robot_state.mode = STATE_AVOID;
    robot_state.state_start_time = millis();
    roll_integral = 0;
    robot_state.climb_phase = 0;
    phase_start_time = 0;
    return;
  }
  
  // ========================================
  // フェーズ0: 0.5秒後退
  // ========================================
  if (robot_state.climb_phase == 0) {
    // フェーズ開始時刻を記録（初回のみ）
    if (phase_start_time == 0) {
      phase_start_time = millis();
      Serial.println(F("CLIMB Phase 0: Reversing"));
    }
    
    // 0.5秒経過したら次のフェーズへ
    if (millis() - phase_start_time > 500) {
      motor_ctrl.stop();
      delay(200);  // 安定化
      robot_state.climb_phase = 1;  // 90度旋回フェーズへ
      phase_start_time = 0;  // リセット
      Serial.println(F("CLIMB Phase 1: Turning left 90 degrees"));
      return;
    }
    
    // 後退
    motor_ctrl.setSpeeds(MOTOR_REVERSE, MOTOR_REVERSE);
    return;
  }
  
  // ========================================
  // フェーズ1: 左に0.5秒旋回
  // ========================================
  if (robot_state.climb_phase == 1) {
    // フェーズ開始時刻を記録（初回のみ）
    if (phase_start_time == 0) {
      phase_start_time = millis();
    }
    
    // 0.5秒旋回
    if (millis() - phase_start_time > 500) {
      motor_ctrl.stop();
      delay(200);  // 安定化
      robot_state.climb_phase = 2;  // 大回りフェーズへ
      phase_start_time = 0;  // リセット
      Serial.println(F("CLIMB Phase 2: Big turn around mountain"));
      return;
    }
    
    // その場で左旋回（左モーター逆転、右モーター正転）
    motor_ctrl.setSpeeds(-MOTOR_AVOID_ROT, MOTOR_AVOID_ROT);
    
    return;
  }
  
  // ========================================
  // フェーズ2: 5秒間平地で大きく右旋回
  // ========================================
  if (robot_state.climb_phase == 2) {
    // フェーズ開始時刻を記録（初回のみ）
    if (phase_start_time == 0) {
      phase_start_time = millis();
    }
    
    // 5秒経過したら次のフェーズへ
    if (millis() - phase_start_time > 5000) {
      motor_ctrl.stop();
      robot_state.climb_phase = 3;  // 向き直しフェーズへ
      phase_start_time = 0;  // リセット
      pi_ctrl.reset();
      return;
    }
    
    // 大きく右旋回（前進しながら右に曲がる）
    // 左モーターを速く、右モーターを遅くする
    motor_ctrl.setSpeeds(200, 105);
    
    // デバッグ出力
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
      Serial.print(F("CLIMB Phase 2 - Time: "));
      Serial.println(millis() - phase_start_time);
      lastDebug = millis();
    }
    
    return;
  }
  
  // ========================================
  // フェーズ3: 右に0.5秒旋回
  // ========================================
  if (robot_state.climb_phase == 3) {
    // フェーズ開始時刻を記録（初回のみ）
    if (phase_start_time == 0) {
      phase_start_time = millis();
    }
    
    // 0.5秒旋回
    if (millis() - phase_start_time > 650) {
      motor_ctrl.stop();
      delay(200);  // 安定化
      robot_state.climb_phase = 4;  // 前進フェーズへ
      phase_start_time = 0;  // リセット
      Serial.println(F("CLIMB Phase 4: Moving forward"));
      return;
    }
    
    // その場で右旋回（左モーター正転、右モーター逆転）
    motor_ctrl.setSpeeds(MOTOR_AVOID_ROT, -MOTOR_AVOID_ROT);
    
    return;
  }
  
  // ========================================
  // フェーズ4: 1秒前進
  // ========================================
  if (robot_state.climb_phase == 4) {
    // フェーズ開始時刻を記録（初回のみ）
    if (phase_start_time == 0) {
      phase_start_time = millis();
    }
    
    // 1秒前進
    if (millis() - phase_start_time > 1000) {
      robot_state.climb_phase = 5;  // 登坂フェーズへ
      phase_start_time = 0;  // リセット
      roll_integral = 0;
      Serial.println(F("CLIMB Phase 5: Starting to climb"));
      return;
    }
    
    // 前進
    motor_ctrl.setSpeeds(MOTOR_FORWARD, MOTOR_FORWARD);
    
    return;
  }
  
  // ========================================
  // フェーズ5: 直進登坂（ロール角制御、カップ検知あり）
  // ========================================
  if (robot_state.climb_phase == 5) {
    // 登頂判定
    if (hasReachedTop()) {
      motor_ctrl.stop();
      Serial.println(F("Reached top!"));
      
      // 登頂後は探索モードへ
      robot_state.mode = STATE_SEARCH;
      robot_state.search_start_time = millis();
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
      
      roll_integral = 0;
      robot_state.climb_phase = 0;
      phase_start_time = 0;
      return;
    }
    
    // カップ検知
    if (dist > 0 && dist < 30) {
      motor_ctrl.stop();
      Serial.println(F("Object detected during climb!"));
      
      robot_state.mode = STATE_CHECK_STATIC;
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
      roll_integral = 0;
      robot_state.climb_phase = 0;
      phase_start_time = 0;
      return;
    }
    
    // ========================================
    // ロール角ベースの直進登坂
    // ========================================
    const int CLIMB_BASE_SPEED = 130;
    
    // 一定間隔で加速度を読み取る
    if (millis() - lastAccelRead > ACCEL_READ_INTERVAL) {
      compass_state.compass.readAcc();
      
      float a_x = compass_state.compass.a.x;
      float a_y = compass_state.compass.a.y;
      float a_z = compass_state.compass.a.z;
      
      float norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
      if (norm < 100) norm = 100;
      
      float a_y_normalized = a_y / norm;
      a_y_normalized = constrain(a_y_normalized, -1.0, 1.0);
      float roll_rad = asin(a_y_normalized);
      current_roll = roll_rad * 180.0 / PI;
      
      lastAccelRead = millis();
    }
    
    // PI制御によるロール角補正
    float roll_error = 0.0 - current_roll;
    const float KP_ROLL = 3.0;
    const float KI_ROLL = 0.01;
    
    float control_u;
    if (abs(roll_error) > 20.0) {
      control_u = KP_ROLL * roll_error;
      roll_integral = 0;
    } else {
      roll_integral += KI_ROLL * roll_error * ACCEL_READ_INTERVAL;
      roll_integral = constrain(roll_integral, -30, 30);
      control_u = KP_ROLL * roll_error + roll_integral;
    }
    
    control_u = constrain(control_u, -80, 80);
    if (abs(roll_error) < 2.0) control_u = 0;
    
    int left = CLIMB_BASE_SPEED + control_u;
    int right = CLIMB_BASE_SPEED - control_u;
    left = constrain(left, 0, 255);
    right = constrain(right, 0, 255);
    
    motor_ctrl.setSpeeds(left, right);
    
    // デバッグ出力
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
      Serial.print(F("CLIMB Phase 4 - ROLL:"));
      Serial.print(current_roll, 1);
      Serial.print(F(" L:"));
      Serial.print(left);
      Serial.print(F(" R:"));
      Serial.println(right);
      lastDebug = millis();
    }
  }
}

// ============================================
// 加速度センサーキャリブレーション（使用しない）
// ============================================
void calibrateAccelZOffset() {
  Serial.println(F("Accel calibration skipped"));
}