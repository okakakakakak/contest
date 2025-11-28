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
  // らせん登頂
  static float pitch_buffer[5] = {0};  // 移動平均用バッファ
  static int buffer_index = 0;

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
    /*if (abs(current_pitch) < SLOPE_PITCH_THRESHOLD / 2.0) {
      flat_detect_count++;*/
    // らせん登頂
    // Pitch角をバッファに保存
    pitch_buffer[buffer_index] = current_pitch;
    buffer_index = (buffer_index + 1) % 5;

    // 移動平均を計算
    float pitch_avg = 0;
    for (int i = 0; i < 5; i++) {
      pitch_avg += pitch_buffer[i];
    }
    pitch_avg /= 5.0;

    // 平地判定（平均値で判定）
    if (abs(pitch_avg) < SLOPE_PITCH_THRESHOLD / 2.0) {
      flat_detect_count++; //ここまでらせん登頂変更
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
// 坂道登坂モードの実行（加速度ベース姿勢制御版）
// ============================================
/**
 * 加速度センサーのロール角を使って横方向の傾きを補正しながら登坂
 * 地磁気センサーに頼らず、加速度センサーのみで姿勢を制御
 */
void runClimbMode() {
  static unsigned long lastAccelRead = 0;
  static float current_roll = 0.0;
  static float roll_integral = 0.0;  // ロール角の積分項
  
  // 超音波センサーで前方の距離を計測
  int dist = ultrasonic.getDistance();
  
  // ========================================
  // 終了条件1: 黒線（坂の頂上やライン）を検出したら停止
  // ========================================
  if (color_sensor.current_color == COLOR_BLACK) {
    motor_ctrl.stop();
    robot_state.mode = STATE_AVOID;
    robot_state.state_start_time = millis();
    roll_integral = 0;  // 積分項をリセット
    return;
  }
  
  // ========================================
  // 終了条件2: 登坂中にカップを検知した場合
  // ========================================
  // 30cm未満の物体を検知したら静止確認へ
  if (dist > 0 && dist < 30) {
    motor_ctrl.stop();
    Serial.println(F("Object detected during climb!"));
    
    robot_state.mode = STATE_CHECK_STATIC;
    robot_state.search_rotation_count = 0;
    robot_state.object_detected_in_search = false;
    roll_integral = 0;  // 積分項をリセット
    return;
  }
  
  // ========================================
  // 終了条件3: 傾斜がなくなった（山を登頂した）場合
  // ========================================
  if (hasReachedTop()) {
    motor_ctrl.stop();
    Serial.println(F("Reached top!"));
    
    // 登頂後は探索モードへ
    robot_state.mode = STATE_SEARCH;
    robot_state.search_start_time = millis();
    robot_state.search_rotation_count = 0;
    robot_state.object_detected_in_search = false;
    
    roll_integral = 0;  // 積分項をリセット
    return;
  }

  // ========================================
  // 登坂継続：ロール角に基づく姿勢制御
  // ========================================
  
  // 坂を登るための基本速度
  const int CLIMB_BASE_SPEED = 170;
  // らせん登頂用
  const int SPIRAL_OFFSET = 60;  // らせんの強さ（旋回半径の調整）大で急旋回、小で緩やか
  
  // ----------------------------------------
  // 方法1: 直進（姿勢制御なし）※コメントアウト
  // ----------------------------------------
  // 左右同じ速度で前進 = 一直線に進む
  // シンプルだが、横方向のズレが蓄積する可能性がある
  // motor_ctrl.setSpeeds(CLIMB_BASE_SPEED, CLIMB_BASE_SPEED);
  // return;  // ← 方法1を使う場合は、この行までコメント解除
  
  // ----------------------------------------
  // 方法2: 加速度ベース姿勢制御（現在使用中）
  // ----------------------------------------
  
  // 一定間隔で加速度を読み取る
  if (millis() - lastAccelRead > ACCEL_READ_INTERVAL) {
    // 加速度センサーから最新の値を読み取る
    compass_state.compass.readAcc();
    
    // 各軸の加速度を取得
    float a_x = compass_state.compass.a.x;
    float a_y = compass_state.compass.a.y;
    float a_z = compass_state.compass.a.z;
    
    // 加速度ベクトルのノルム（大きさ）を計算
    float norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    
    // ゼロ除算を防ぐ
    if (norm < 100) {
      norm = 100;
    }
    
    // 加速度を正規化
    float a_y_normalized = a_y / norm;
    
    // ロール角を計算（ラジアン）
    // PDFの式に基づく：Roll = arcsin(a_y_normalized)
    a_y_normalized = constrain(a_y_normalized, -1.0, 1.0);
    float roll_rad = asin(a_y_normalized);
    
    // 度に変換
    current_roll = roll_rad * 180.0 / PI;
    
    lastAccelRead = millis();
    
    // デバッグ出力（500msごと）
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
      Serial.print(F("CLIMB - ROLL:"));
      Serial.print(current_roll, 1);
      Serial.print(F(" A_Y:"));
      Serial.println((int)a_y);
      lastDebug = millis();
    }
  }
  
  // ========================================
  // PI制御によるロール角補正
  // ========================================
  float control_yaw = 0; //らせん登頂追加　PI制御関連なのでmotion_controlに統合すべき
  // ロール角の目標値は0度（まっすぐ）
  float roll_error = 0.0 - current_roll;
  
  // PI制御のパラメータ
  const float KP_ROLL = 3.0;      // 比例ゲイン
  const float KI_ROLL = 0.01;     // 積分ゲイン
  
  float control_u;
  
  if (abs(roll_error) > 20.0) {
    // 誤差が大きい場合：比例制御のみ（高速補正）
    control_u = KP_ROLL * roll_error;
    roll_integral = 0;  // 積分項をリセット
  } else {
    // 誤差が小さい場合：PI制御（精密制御）
    // 積分項を更新
    roll_integral += KI_ROLL * roll_error * ACCEL_READ_INTERVAL;
    
    // アンチワインドアップ
    roll_integral = constrain(roll_integral, -30, 30);
    
    // 制御入力の計算
    control_u = KP_ROLL * roll_error + roll_integral;
  }
  
  // 制御入力を制限
  control_u = constrain(control_u, -80, 80);
  
  // 誤差が非常に小さい場合は補正しない（±2度以内）
  if (abs(roll_error) < 2.0) {
    control_u = 0;
  }
  
  // ========================================
  // らせん登頂追加分（Yaw補正）
  // ========================================
  compass_state.updateHeading(MAGNETIC_DECLINATION);
  float current_heading = compass_state.current_heading;
  float yaw_error = TARGET_HEADING - current_heading;
  if (yaw_error > 180) yaw_error -= 360;
  if (yaw_error < -180) yaw_error += 360;

  control_yaw = pi_ctrl.kp * yaw_error + pi_ctrl.ti_inv * pi_ctrl.sum_e;
  pi_ctrl.sum_e += yaw_error;
  control_yaw = constrain(control_yaw, -40, 40);

  
  // ========================================
  // モーター速度の計算
  // ========================================
  // ロール角が正（右に傾いている）→ 左モーターを速くして左に曲がる
  // ロール角が負（左に傾いている）→ 右モーターを速くして右に曲がる
  
  //int left = CLIMB_BASE_SPEED + control_u;
  //int right = CLIMB_BASE_SPEED - control_u;
  //らせん登頂用(左カーブ)
  int left = CLIMB_BASE_SPEED - SPIRAL_OFFSET + control_u + control_yaw;
  int right = CLIMB_BASE_SPEED + SPIRAL_OFFSET - control_u - control_yaw;

  
  // スピードを制限（後退しないように0以上に制限）
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  
  motor_ctrl.setSpeeds(left, right);
  
  // デバッグ出力（制御量とモーター速度）
  static unsigned long lastDebug2 = 0;
  if (millis() - lastDebug2 > 500) {
    Serial.print(F("CLIMB - ERROR:"));
    Serial.print(roll_error, 1);
    Serial.print(F(" U:"));
    Serial.print(control_u, 1);
    Serial.print(F(" L:"));
    Serial.print(left);
    Serial.print(F(" R:"));
    Serial.println(right);
    lastDebug2 = millis();
  }
}
