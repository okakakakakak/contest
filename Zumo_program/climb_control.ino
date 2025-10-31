#include "definitions.h"

// ============================================
// 加速度センサーによる傾斜検知
// ============================================
/**
 * 坂道（傾斜）を検知する関数
 * 加速度センサーのZ軸の値を読み取り、閾値と比較して傾斜を判定する
 * 
 * @return true: 坂道を検知した / false: 平坦な地面
 */
bool isSlopeDetected() {
  // LSM303の加速度計はZumo_program.inoでcompass_state.compass.enableDefault()により初期化済み

  // 前回の加速度読み取り時刻を記録（頻繁な読み取りを避けるため）
  static unsigned long lastAccelRead = 0;
  // 現在のZ軸加速度値を保持
  static int current_accel_z = 0;
  
  // 頻繁に読み取ると動作に影響するため、一定間隔(ACCEL_READ_INTERVAL)で読み取る
  if (millis() - lastAccelRead > ACCEL_READ_INTERVAL) {
    // 加速度センサーから最新の値を読み取る
    compass_state.compass.readAcc();
    
    // Z軸の値を読み取り、オフセットを適用
    // 坂を登る場合、Z軸の値が変化するため、その変化を捉える
    // オフセットを加えることで、水平時に0になるように補正
    current_accel_z = compass_state.compass.a.z + ACCEL_Z_OFFSET;
    
    // 最後に読み取った時刻を更新
    lastAccelRead = millis();
  }
  
  // Z軸の値の絶対値が閾値(SLOPE_THRESHOLD)を超えていれば坂道と判定
  // 正の傾斜(上り坂)でも負の傾斜(下り坂)でも検知できるようabs()を使用
  if (abs(current_accel_z) > SLOPE_THRESHOLD) {
    return true;  // 坂道を検知
  }
  return false;  // 平坦な地面
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
  if (!isSlopeDetected()) {
    motor_ctrl.stop();  // モーターを停止
    
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
// Z軸オフセット・キャリブレーション
// ============================================
/**
 * 加速度センサーのZ軸オフセットをキャリブレーションする
 * 水平な場所に置いた状態で実行し、Z軸が0になるように補正値を計算する
 * この関数はsetup()で一度だけ実行される
 */
void calibrateAccelZOffset() {
  // 読み取り回数（多いほど精度が上がる）
  const int NUM_READINGS = 100;
  
  // Z軸の値の合計を計算するための変数
  long sum_z = 0;
  
  // 100回Z軸の値を読み取り、合計を計算
  for (int i = 0; i < NUM_READINGS; i++) {
    compass_state.compass.readAcc();  // 加速度を読み取る
    sum_z += compass_state.compass.a.z;  // Z軸の値を加算
    delay(5);  // 読み取り間隔（センサーの安定化のため）
  }
  
  // オフセット（水平な時の読み取り値）を計算
  // 理想値 0 からの差分をオフセットとする
  // ( sum_z / NUM_READINGS ) が水平時の平均読み取り値
  // マイナスを付けることで、補正時に0になるようにする
  ACCEL_Z_OFFSET = -(sum_z / NUM_READINGS); 
  
  // 💡 ACCEL_Z_OFFSET は、climb_control.inoのisSlopeDetected()で以下のように使用されます。
  // current_accel_z = compass_state.compass.a.z + ACCEL_Z_OFFSET;
  // -> 水平な時、current_accel_z は約 0 になる
}