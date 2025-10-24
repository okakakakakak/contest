#include "definitions.h"

// ============================================
// 加速度センサーによる傾斜検知
// ============================================
bool isSlopeDetected() {
  // LSM303の加速度計はZumo_program.inoでcompass_state.compass.enableDefault()により初期化済み

  static unsigned long lastAccelRead = 0;
  static int current_accel_z = 0;
  
  // 頻繁に読み取ると動作に影響するため、一定間隔で読み取る
  if (millis() - lastAccelRead > ACCEL_READ_INTERVAL) {
    compass_state.compass.readAcc();
    // Z軸の値を読み取り、オフセットを適用
    // 坂を登る場合、Z軸の値が変化するため、その変化を捉える
    current_accel_z = compass_state.compass.a.z + ACCEL_Z_OFFSET;
    lastAccelRead = millis();
  }
  
  // Z軸の値が閾値を超えていれば坂道と判定
  if (abs(current_accel_z) > SLOPE_THRESHOLD) {
    return true;
  }
  return false;
}

// ============================================
// 坂道登坂モードの実行
// ============================================
void runClimbMode() {
  int dist = ultrasonic.getDistance(); // 距離を計測
  
  // 終了条件1: 黒線（坂の頂上やライン）を検出したら停止
  if (color_sensor.current_color == COLOR_BLACK) {
    motor_ctrl.stop();
    robot_state.mode = STATE_AVOID; // STATE_STOPではなくAVOIDでライン回避
    robot_state.state_start_time = millis();
    return;
  }
  
  // 💡 終了条件2: 傾斜がなくなった（山を登頂した）場合に宝を検知し遷移
  if (!isSlopeDetected()) {
    motor_ctrl.stop();
    
    // 宝の検知条件：距離が30cm未満
    if (dist > 0 && dist < 30) { 
      // 登頂成功後、宝を検知 → STATE_CHECK_STATIC を経由して STATE_APPROACH へ
      robot_state.mode = STATE_CHECK_STATIC;
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
    } else {
      // 登頂成功したが宝は検知せず → STATE_SEARCH へ戻る
      robot_state.mode = STATE_SEARCH;
      robot_state.search_start_time = millis();
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
    }
    pi_ctrl.reset();
    return;
  }

  // 目標方位(TARGET_HEADING)に向かってPI制御を行う (登坂継続)
  // PI制御による方位維持は motion_control.ino の turnTo() 関数を使用
  float control_u = turnTo(TARGET_HEADING);
  
  // 制御入力が小さい場合は中央値を0にする（モーターの遊び対策）
  if (abs(control_u) < 5) {
    control_u = 0;
  }
  
  // 左右のスピードを計算（基本速度 + 制御入力）
  const int CLIMB_BASE_SPEED = 180; // 坂を登るための高い基本速度
  
  int left = CLIMB_BASE_SPEED + control_u * 0.5;
  int right = CLIMB_BASE_SPEED - control_u * 0.5;
  
  // スピードを制限（オーバーフロー・モーター保護）
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  motor_ctrl.setSpeeds(left, right);
}

// ============================================
// Z軸オフセット・キャリブレーション
// ============================================
void calibrateAccelZOffset() {
  const int NUM_READINGS = 100; // 読み取り回数
  long sum_z = 0;
  
  // 100回Z軸の値を読み取り、合計を計算
  for (int i = 0; i < NUM_READINGS; i++) {
    compass_state.compass.readAcc();
    sum_z += compass_state.compass.a.z;
    delay(5); // 読み取り間隔
  }
  
  // オフセット（水平な時の読み取り値）を計算
  // 理想値 0 からの差分をオフセットとする
  // ( sum_z / NUM_READINGS ) が水平時の平均読み取り値
  ACCEL_Z_OFFSET = -(sum_z / NUM_READINGS); 
  
  // 💡 ACCEL_Z_OFFSET は、climb_control.inoのisSlopeDetected()で以下のように使用されます。
  // current_accel_z = compass_state.compass.a.z + ACCEL_Z_OFFSET;
  // -> 水平な時、current_accel_z は約 0 になる
}