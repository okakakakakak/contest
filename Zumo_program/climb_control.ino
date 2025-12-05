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
// 坂道登坂モードの実行（後退→大回り→登坂→頂上→下山）
// ============================================
void runClimbMode() {
  static unsigned long lastAccelRead = 0;
  static float current_roll = 0.0;
  static float roll_integral = 0.0;  // ロール角の積分項
  static unsigned long phase_start_time = 0; // 各フェーズの開始時刻
  
  // 超音波センサーで前方の距離を計測
  int dist = ultrasonic.getDistance();

  // ========================================
  // 終了条件: 黒線を検出したら即時停止（フェーズ問わず）
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
    if (phase_start_time == 0) {
      phase_start_time = millis();
      Serial.println(F("CLIMB Phase 0: Reversing"));
    }
    
    if (millis() - phase_start_time > 250) {
      motor_ctrl.stop();
      delay(200);
      robot_state.climb_phase = 1;
      phase_start_time = 0;
      return;
    }
    motor_ctrl.setSpeeds(MOTOR_REVERSE, MOTOR_REVERSE);
    return;
  }
  
  // ========================================
  // フェーズ1: 左に0.5秒旋回
  // ========================================
  if (robot_state.climb_phase == 1) {
    if (phase_start_time == 0) phase_start_time = millis();
    
    if (millis() - phase_start_time > 250) {
      motor_ctrl.stop();
      delay(200);
      robot_state.climb_phase = 2;
      phase_start_time = 0;
      Serial.println(F("CLIMB Phase 2: Big turn"));
      return;
    }
    motor_ctrl.setSpeeds(-MOTOR_AVOID_ROT, MOTOR_AVOID_ROT);
    return;
  }
  
  /*
  // ========================================
  // フェーズ2: 5秒間平地で大きく右旋回
  // ========================================
  if (robot_state.climb_phase == 2) {
    if (phase_start_time == 0) phase_start_time = millis();
    
    if (millis() - phase_start_time > 5000) {
      motor_ctrl.stop();
      robot_state.climb_phase = 3;
      phase_start_time = 0;
      return;
    }
    motor_ctrl.setSpeeds(200, 105);
    return;
  }*/
// フェーズ2: ふもとを大きく右旋回し、機体の右側がTARGET_HEADINGを向いたら次へ
if (robot_state.climb_phase == 2) {
    if (phase_start_time == 0) {
        phase_start_time = millis();
        Serial.println(F("CLIMB Phase 2: Big right turn to align side with TARGET_HEADING"));
    }

    // 常に大きく右旋回（左速い、右遅い）
    motor_ctrl.setSpeeds(200, 105);

    // 機体の右側がTARGET_HEADINGを向いているか判定
    // → 現在の方位 +90° が TARGET_HEADING に近いかどうか
    float side_heading = compass_state.current_heading + 90.0;
    if (side_heading >= 360.0) side_heading -= 360.0;

    float heading_error = TARGET_HEADING - side_heading;
    while (heading_error < -180) heading_error += 360;
    while (heading_error > 180) heading_error -= 360;

    // 誤差が一定以下なら次フェーズへ
    if (abs(heading_error) < 10.0) {  // 10°以内で判定
        motor_ctrl.stop();
        robot_state.climb_phase = 3;  // 向き調整フェーズへ
        phase_start_time = 0;
        Serial.println(F("Right side aligned with TARGET_HEADING. Moving to Phase 3."));
    }
    return;
}


  
  // ========================================
  // フェーズ3: 右に0.5秒旋回（向き調整）
  // ========================================
  if (robot_state.climb_phase == 3) {
    if (phase_start_time == 0) phase_start_time = millis();
    
    if (millis() - phase_start_time > 250) {
      motor_ctrl.stop();
      delay(200);
      robot_state.climb_phase = 4;
      phase_start_time = 0;
      Serial.println(F("CLIMB Phase 4: Move forward"));
      return;
    }
    motor_ctrl.setSpeeds(MOTOR_AVOID_ROT, -MOTOR_AVOID_ROT);
    return;
  }
  
  // ========================================
  // フェーズ4: 1秒前進（坂への助走）
  // ========================================
  if (robot_state.climb_phase == 4) {
    if (phase_start_time == 0) phase_start_time = millis();
    
    if (millis() - phase_start_time > 1000) {
      robot_state.climb_phase = 5;
      phase_start_time = 0;
      roll_integral = 0;
      Serial.println(F("CLIMB Phase 5: Climbing UP"));
      return;
    }
    motor_ctrl.setSpeeds(MOTOR_FORWARD, MOTOR_FORWARD);
    return;
  }
  
  // ========================================
  // フェーズ5: 直進登坂（ロール角制御）
  // ========================================
  if (robot_state.climb_phase == 5) {
    // --- 変更点: 登頂したらフェーズ6（頂上移動）へ ---
    if (hasReachedTop()) {
      // 停止せずにそのまま次のフェーズへ
      robot_state.climb_phase = 6;
      phase_start_time = 0;
      roll_integral = 0; // 積分リセット
      Serial.println(F("Reached Top! Moving to traverse plateau."));
      return;
    }
    
    // カップ検知時は一時停止・確認（既存ロジック）
    if (dist > 0 && dist < 30) {
      motor_ctrl.stop();
      robot_state.mode = STATE_CHECK_STATIC;
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
      roll_integral = 0;
      robot_state.climb_phase = 0;
      phase_start_time = 0;
      return;
    }
    
    // ロール角制御を実行（共通処理として後述のブロックを使用）
    executeRollControl(lastAccelRead, current_roll, roll_integral);
  }

  // ========================================
  // フェーズ6: 頂上の平地を直進（下り坂を探す）
  // ========================================
  if (robot_state.climb_phase == 6) {
    // 単純直進（頂上は平らと仮定）
    motor_ctrl.setSpeeds(MOTOR_FORWARD, MOTOR_FORWARD);

    // 坂道を検知したら「下り坂」と判断してフェーズ7へ
    // isSlopeDetectedは絶対値で判定するため、下りも検知可能
    if (isSlopeDetected()) {
      robot_state.climb_phase = 7;
      phase_start_time = 0;
      roll_integral = 0;
      Serial.println(F("Descent detected! Starting descent."));
      return;
    }

    // 万が一、5秒以上平地が続いたら（頂上が非常に広い、または検知ミス）、強制的に探索へ
    static unsigned long traverseStart = 0;
    if (traverseStart == 0) traverseStart = millis();
    if (millis() - traverseStart > 5000) {
        motor_ctrl.stop();
        robot_state.mode = STATE_SEARCH;
        robot_state.climb_phase = 0;
        traverseStart = 0;
        Serial.println(F("Timeout on top. Searching."));
    }
  }

  // ========================================
  // フェーズ7: 下り坂（ロール角制御で直進）
  // ========================================
  if (robot_state.climb_phase == 7) {
    // 平地（下山完了）を検知したら終了
    if (hasReachedTop()) { // hasReachedTopは「平地検知」関数なので下山完了判定にも使える
      motor_ctrl.stop();
      Serial.println(F("Descended to flat ground. To SEARCH mode."));
      
      // 探索モードへ遷移
      robot_state.mode = STATE_SEARCH;
      robot_state.search_start_time = millis();
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
      
      // リセット
      roll_integral = 0;
      robot_state.climb_phase = 0;
      phase_start_time = 0;
      return;
    }

    // 下り坂もロール角制御を使って真っ直ぐ降りる
    executeRollControl(lastAccelRead, current_roll, roll_integral);
  }
}

// ============================================
// 補助関数: ロール角制御（フィルタ・PI制御強化版）
// ============================================
void executeRollControl(unsigned long &lastAccelRead, float &current_roll, float &roll_integral) {
    // 基本速度（坂道なので少しパワーが必要）
    const int CLIMB_BASE_SPEED = 150; 
    
    // 制御ゲイン（要調整）
    // PDFのP制御/PI制御の解説 [cite: 208, 213] を参考に設定
    const float KP_ROLL = 4.0;   // 比例ゲイン
    const float KI_ROLL = 1.5;   // 積分ゲイン（単位を秒にしたため値を変更）
    const float DT_SEC = 0.05;   // 制御周期 (50ms = 0.05s)

    // フィルタ係数（0.0〜1.0）: 小さいほど滑らかだが遅れが生じる
    const float FILTER_ALPHA = 0.2; 
    static float filtered_ay = 0.0;
    static float filtered_az = 0.0;
    static bool filter_initialized = false;

    // 一定間隔(ACCEL_READ_INTERVAL = 50ms)で加速度を読み取る
    if (millis() - lastAccelRead > ACCEL_READ_INTERVAL) {
        compass_state.compass.readAcc();
        
        // 1. 生データの取得
        float raw_ay = compass_state.compass.a.y;
        float raw_az = compass_state.compass.a.z;

        // 2. ローパスフィルタ（振動対策）
        if (!filter_initialized) {
            filtered_ay = raw_ay;
            filtered_az = raw_az;
            filter_initialized = true;
        } else {
            // 前回の値と今回の値を混ぜて滑らかにする
            filtered_ay = filtered_ay * (1.0 - FILTER_ALPHA) + raw_ay * FILTER_ALPHA;
            filtered_az = filtered_az * (1.0 - FILTER_ALPHA) + raw_az * FILTER_ALPHA;
        }

        // 3. ロール角の計算
        // PDF 8-4.2.2項  に基づき、ayは sin(roll) に比例
        // Z軸の値も考慮して正規化する（簡易的にYとZのみで計算）
        float norm = sqrt(filtered_ay * filtered_ay + filtered_az * filtered_az);
        if (norm < 100) norm = 100; // ゼロ除算防止

        float ay_normalized = filtered_ay / norm;
        ay_normalized = constrain(ay_normalized, -1.0, 1.0);
        
        // 角度計算（ラジアン → 度）
        float roll_rad = asin(ay_normalized);
        current_roll = roll_rad * 180.0 / PI;

        lastAccelRead = millis();
    }
    
    // ========================================
    // PI制御 (PDF 8-4.4項 [cite: 211-218] 参照)
    // ========================================
    
    // 目標値は 0.0度（水平）
    // エラー = 目標 - 現在値
    float roll_error = 0.0 - current_roll;

    float control_u = 0;

    // 不感帯（2度未満の傾きは無視してハンチング防止）
    if (abs(roll_error) < 2.0) {
        // 誤差が小さいときは積分項を増やさない（またはリセットも検討）
        control_u = 0;
    } else {
        // 積分項の計算: 誤差 * 時間(秒)
        // PDF [cite: 213] の積分動作に対応
        roll_integral += roll_error * DT_SEC;
        
        // アンチワインドアップ（積分の暴走を防ぐ制限）
        roll_integral = constrain(roll_integral, -15.0, 15.0);

        // 操作量の計算 u = Kp*e + Ki*∫e
        control_u = (KP_ROLL * roll_error) + (KI_ROLL * roll_integral);
    }
    
    // 操作量の制限（最大速度差）
    control_u = constrain(control_u, -100, 100);
    
    // ========================================
    // モーター出力の決定
    // ========================================
    // PDF座標系 [cite: 134] では Y軸は「左」
    // 右に傾く → ay > 0 → current_roll > 0 → error < 0 → control_u < 0
    // 姿勢を戻すには「左」に旋回したい（左を減速、右を加速）
    
    // control_u が負のとき:
    // Left = 150 + (-val) = 減速
    // Right = 150 - (-val) = 加速
    // → 左旋回となるため、この符号で正しい。
    
    // もし実機で逆に動く（転倒する）場合は、ここを逆にしてください：
    // int left = CLIMB_BASE_SPEED - control_u;
    // int right = CLIMB_BASE_SPEED + control_u;
    
    int left = CLIMB_BASE_SPEED + control_u;
    int right = CLIMB_BASE_SPEED - control_u;

    // モーター速度の正規化
    left = constrain(left, 0, 255);
    right = constrain(right, 0, 255);
    
    motor_ctrl.setSpeeds(left, right);

    // デバッグ用（シリアルプロッタで見ると挙動がわかります）
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 200) {
        Serial.print(F("ROLL_CTRL: Roll:"));
        Serial.print(current_roll);
        Serial.print(F(" Err:"));
        Serial.print(roll_error);
        Serial.print(F(" U:"));
        Serial.println(control_u);
        lastDebug = millis();
    }
}

// ============================================
// 加速度センサーキャリブレーション（使用しない）
// ============================================
void calibrateAccelZOffset() {
  Serial.println(F("Accel calibration skipped"));
}