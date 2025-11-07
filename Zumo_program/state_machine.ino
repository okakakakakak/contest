/*
 * state_machine.ino
 * 
 * 【機能概要】
 * ロボットの状態遷移を管理するメインモジュール
 * 
 * 【主要機能】
 * 1. task(): 状態に応じた動作を実行（状態マシン）
 * 2. printModeChange(): モード変更を表示
 * 3. printStatus(): センサー情報とモーター速度を表示
 * 
 * 【状態遷移の概要】
 * INIT → SEARCH → CHECK_STATIC → APPROACH → TURN_TO_TARGET → 
 * WAIT_AFTER_TURN → ESCAPE → DEPOSIT → SEARCH...
 * 
 * ※黒線検知時はAVOID、坂道検知時はCLIMBに遷移
 */

#include "definitions.h"

// ============================================
// モード名取得（PROGMEM使用でRAM節約）
// ============================================
// 文字列をプログラムメモリに格納（RAMを節約）
const char str_init[] PROGMEM = "INIT";
const char str_direction[] PROGMEM = "DIRECTION";
const char str_search[] PROGMEM = "SEARCH";
const char str_check[] PROGMEM = "CHECK_STATIC";
const char str_approach[] PROGMEM = "APPROACH";
const char str_turn[] PROGMEM = "TURN_TO_TARGET";
const char str_wait[] PROGMEM = "WAIT_AFTER_TURN";
const char str_escape[] PROGMEM = "ESCAPE";
const char str_avoid[] PROGMEM = "AVOID";
const char str_stop[] PROGMEM = "STOP";
const char str_move[] PROGMEM = "MOVE";
const char str_climb[] PROGMEM = "CLIMB";
const char str_check_zone[] PROGMEM = "CHECK_ZONE";
const char str_deposit[] PROGMEM = "DEPOSIT";
const char str_unknown[] PROGMEM = "UNKNOWN";

// モード名の配列（プログラムメモリに格納）
const char* const mode_names[] PROGMEM = {
  str_init, str_direction, str_search, str_check, str_approach, str_turn,
  str_wait, str_escape, str_avoid, str_stop, str_move,
  str_climb,
  str_check_zone, 
  str_deposit
};

/**
 * モード名を表示する関数
 * プログラムメモリから文字列を読み出して表示
 * 
 * @param mode モード番号
 */
void printModeName(byte mode) {
  if (mode < 14) {
    // プログラムメモリから文字列をバッファにコピー
    char buffer[20];
    strcpy_P(buffer, (char*)pgm_read_word(&(mode_names[mode])));
    Serial.print(buffer);
  } else {
    Serial.print(F("UNKNOWN"));
  }
}

// ============================================
// モード変更通知
// ============================================
/**
 * モードが変更された時に通知を表示する関数
 * 前回のモードと現在のモードを比較し、変更があれば表示
 */
void printModeChange() {
  // モードが変更されたかチェック
  if (robot_state.mode != robot_state.previous_mode) {
    // ">>> 前のモード -> 新しいモード" の形式で表示
    Serial.print(F(">>> "));
    printModeName(robot_state.previous_mode);
    Serial.print(F(" -> "));
    printModeName(robot_state.mode);
    Serial.println();
    
    // 前回のモードを更新
    robot_state.previous_mode = robot_state.mode;
  }
}

// ============================================
// ステータス表示（500msごと、簡略化）
// ============================================
/**
 * ロボットのステータスを定期的に表示する関数
 * 500msごとに、モード、距離、色、方位、モーター速度を表示
 */
void printStatus() {
  // 前回表示した時刻を記録
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();

  // 500ms経過していなければ何もしない
  if (currentTime - lastPrintTime >= 500) {
    // ========================================
    // センサー情報を取得
    // ========================================
    int dist = ultrasonic.getDistance();           // 距離（cm）
    int colorCode = color_sensor.current_color;    // 色コード
    float heading = compass_state.current_heading; // 方位角（度）
    int motorL = motor_ctrl.left_speed;            // 左モーター速度
    int motorR = motor_ctrl.right_speed;           // 右モーター速度

    // ========================================
    // モード名を表示
    // ========================================
    Serial.print("MODE:");
    printModeName(robot_state.mode);
    Serial.println();

    // ========================================
    // 距離を表示
    // ========================================
    Serial.print("DIST:");
    Serial.println(dist);

    // ========================================
    // 色を表示（数値 → 文字列変換）
    // ========================================
    Serial.print("COLOR:");
    switch (colorCode) {
      case COLOR_WHITE: Serial.println("WHITE"); break;
      case COLOR_RED:   Serial.println("RED");   break;
      case COLOR_BLACK: Serial.println("BLACK"); break;
      case COLOR_BLUE:  Serial.println("BLUE");  break;
      default:          Serial.println("OTHER"); break;
    }

    // ========================================
    // 方位を表示（必要なモードのみ）
    // ========================================
    // 旋回中や脱出中のみ方位を表示（通信量削減）
    if (robot_state.mode == STATE_TURN_TO_TARGET || robot_state.mode == STATE_ESCAPE) {
      Serial.print("HEADING:");
      Serial.println(heading, 0);  // 小数点なしで送信
    }

    // ========================================
    // モーター速度を表示
    // ========================================
    Serial.print("MOTOR:");
    Serial.print(motorL);
    Serial.print(",");
    Serial.println(motorR);

    // ========================================
    // 加速度を表示
    // ========================================
    // 加速度センサ（X, Y, Z）
    compass_state.compass.readAcc();  // LSM303から加速度取得
    int ax = compass_state.compass.a.x;
    int ay = compass_state.compass.a.y;
    int az = compass_state.compass.a.z;

    Serial.print("ACCEL:");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.println(az);

    // 最後に表示した時刻を更新
    lastPrintTime = currentTime;
  }
}


// ============================================
// メインタスク（状態遷移）
// ============================================
/**
 * ロボットのメイン制御ループ
 * 現在の状態に応じて適切な動作を実行し、状態遷移を管理
 * 
 * 【呼び出し】
 * loop()から毎回呼び出される
 */
void task() {
  // ========================================
  // 初期処理
  // ========================================
  robot_state.updateTime();  // 時刻を更新
  printModeChange();         // モード変更を表示
  
  // 距離を計測
  int dist = ultrasonic.getDistance();
  
  // ========================================
  // 状態に応じた処理を実行
  // ========================================
  switch (robot_state.mode) {
    
    // ========================================
    // STATE_INIT: 初期化状態
    // ========================================
    case STATE_INIT:
      // 探索モードに遷移 💡 STATE_DIRECTION に変更
      robot_state.mode = STATE_DIRECTION;
      robot_state.state_start_time = millis(); // 💡 新しい状態の開始時刻を記録
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
      pi_ctrl.reset();  // PI制御をリセット
      break;

    // ========================================
    // STATE_DIRECTION: 💡 目標方向の反対（TARGET_HEADING+180°）を向く（一度きり）
    // ========================================
    case STATE_DIRECTION: {
      // 旋回後の直進開始時刻を記録するstatic変数
      static unsigned long straight_start_time = 0;
      
      // 最終的な目標方位を計算 (TARGET_HEADING + 180°)
      float target_reverse_heading = TARGET_HEADING + 180.0;
      if (target_reverse_heading >= 360.0) {
        target_reverse_heading -= 360.0;
      }
    
      // 黒線・赤色・青色を検知したら回避モードへ
      if (color_sensor.current_color == COLOR_BLACK ||
      color_sensor.current_color == COLOR_RED ||
      color_sensor.current_color == COLOR_BLUE) {
      robot_state.mode = STATE_AVOID;
      robot_state.state_start_time = millis();
      }
      
    // ========================================
    // サブステップ 1: 弧を描く旋回
    // ========================================
    if (straight_start_time == 0) {
        // 最初の100msは安定化のために停止
        if (millis() - robot_state.state_start_time < 100) {
          motor_ctrl.stop();
          break;
        }
          
        // PI制御で目標方位への制御入力を計算
        float u = turnTo(target_reverse_heading);
          
        // 方位角誤差を計算
        float heading_error = target_reverse_heading - compass_state.current_heading;
        while (heading_error < -180) heading_error += 360;
        while (heading_error > 180) heading_error -= 360;

        // 💡 旋回速度の基本値 (弧を描くための前進成分)
        // const int BASE_TURN_SPEED = 50; 
          
        // 制御入力 u を使って左右のモーター速度を計算
        // 弧を描く旋回: (基本速度 + 制御) / (基本速度 - 制御)
        int left = constrain(MOTOR_TURN + u * 0.5, 0, 130);
        int right = constrain(MOTOR_TURN - u * 0.5, 0, 130);
          
        // 💡 旋回完了判定（誤差5度未満）
        if (abs(u) < 2 || abs(heading_error) < 5.0) {
            motor_ctrl.stop();
            straight_start_time = millis(); // 直進開始時刻を記録
            break;
        }
          
        motor_ctrl.setSpeeds(left, right);
          
        // タイムアウト（5秒）したら強制的に直進ステップへ
        if (millis() - robot_state.state_start_time > 5000) {
            motor_ctrl.stop();
            straight_start_time = millis();
        }
        break;
    }
      
    // ========================================
    // サブステップ 2: 直進（2秒間）
    // ========================================
    if (millis() - straight_start_time < 2000) {
        // 2秒間前進
        motor_ctrl.setSpeeds(MOTOR_MOVE, MOTOR_MOVE); 
        break;
    }

    // ========================================
    // サブステップ 3: 完了
    // ========================================
    motor_ctrl.stop();
    // 旋回+直進完了 → STATE_SEARCH に遷移
    robot_state.mode = STATE_SEARCH;
    robot_state.search_start_time = millis();
    robot_state.search_rotation_count = 0;
    robot_state.object_detected_in_search = false;
    pi_ctrl.reset();
      
    // straight_start_timeをリセットして次の実行に備える（このモードは一度きりだが念のため）
    straight_start_time = 0; 
    break;
  }

    // ========================================
    // STATE_SEARCH: 探索状態
    // ========================================
    case STATE_SEARCH: {
      // 物体検知ロジック：30cm未満の物体を3回検知したら静止確認へ
      if (dist > 0 && dist < 30) {
        // 初めて物体を検知した場合
        if (!robot_state.object_detected_in_search) {
          robot_state.object_detected_in_search = true;
          robot_state.search_rotation_count = 0;
        }
        
        // 検知回数をカウント
        robot_state.search_rotation_count++;
        
        // 3回検知したら静止確認へ
        if (robot_state.search_rotation_count >= 3) {
          motor_ctrl.stop();
          delay(100);
          robot_state.mode = STATE_CHECK_STATIC;
          robot_state.search_rotation_count = 0;
          robot_state.object_detected_in_search = false;
          break;
        }
      } else {
        // 物体を見失った場合、カウントをリセット
        if (robot_state.object_detected_in_search && robot_state.search_rotation_count < 3) {
          robot_state.search_rotation_count = 0;
          robot_state.object_detected_in_search = false;
        }
      }
      
      // 反時計回りに回転（左モーター逆転、右モーター正転）
      motor_ctrl.setSpeeds(-MOTOR_ROTATE, MOTOR_ROTATE);
      
      // 💡 修正箇所：時間が短くなる問題を防ぐためのロジック
      // 探索開始からの経過時間で STATE_MOVE に遷移
      if (millis() - robot_state.search_start_time > 5000) {
        // 5秒経過したら移動モードへ
        // 🚨 意図しないリセットを防ぐため、オブジェクト検知フラグも確認
        motor_ctrl.stop();
        
        // STATE_MOVEへ遷移
        robot_state.mode = STATE_MOVE;
        robot_state.state_start_time = millis();  // STATE_MOVEの開始時間を適切に記録
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      }
      break;
    }

    // ========================================
    // STATE_MOVE: 移動状態
    // ========================================
    case STATE_MOVE:
      // 前進
      motor_ctrl.setSpeeds(MOTOR_MOVE, MOTOR_MOVE);
      
      // 💡 NEW: 傾斜検知による STATE_CLIMB への遷移
      if (isSlopeDetected()) {
        motor_ctrl.stop();
        robot_state.mode = STATE_CLIMB;
        robot_state.state_start_time = millis();
        pi_ctrl.reset();
        break;
      }

      // 黒線・赤色・青色を検知したら回避モードへ
      if (color_sensor.current_color == COLOR_BLACK ||
      color_sensor.current_color == COLOR_RED ||
      color_sensor.current_color == COLOR_BLUE) {
      robot_state.mode = STATE_AVOID;
      robot_state.state_start_time = millis();
      }
      // 30cm未満の物体を検知したら静止確認へ
      else if (dist > 0 && dist < 30) {
        motor_ctrl.stop();
        robot_state.mode = STATE_CHECK_STATIC;
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      } 
      // 2秒経過したら探索モードへ
      else if (millis() - robot_state.state_start_time > 2000) {
        motor_ctrl.stop();
        robot_state.mode = STATE_SEARCH;
        robot_state.search_start_time = millis();
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      }
      break;

    // ========================================
    // STATE_CLIMB: 💡 NEW: 坂道登坂モード
    // ========================================
    case STATE_CLIMB:
      // climb_control.ino の関数を呼び出し
      runClimbMode();
      break;
      
    // ========================================
    // STATE_CHECK_STATIC: 静止物体確認状態
    // ========================================
    case STATE_CHECK_STATIC:
      // 物体が静止しているか判定
      if (ultrasonic.isObjectStatic()) {
        // 静止している → 接近モードへ
        robot_state.mode = STATE_APPROACH;
      } else {
        // 動いている（動物など） → 探索モードへ戻る
        robot_state.mode = STATE_SEARCH;
        robot_state.search_start_time = millis();
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      }
      break;

    // ========================================
    // STATE_APPROACH: 接近状態
    // ========================================
    case STATE_APPROACH:
    // 黒線・赤色・青色を検知したら回避モードへ
    if (color_sensor.current_color == COLOR_BLACK ||
      color_sensor.current_color == COLOR_RED ||
      color_sensor.current_color == COLOR_BLUE) {
      robot_state.mode = STATE_AVOID;
      robot_state.state_start_time = millis();
      break;
      }

      // 前進
      motor_ctrl.setSpeeds(MOTOR_FORWARD, MOTOR_FORWARD);
      
      // 7cm未満に近づいたら旋回モードへ
      if (dist < 7) {
        robot_state.mode = STATE_TURN_TO_TARGET;
        robot_state.state_start_time = millis();
        pi_ctrl.reset();
      }
      break;

// ========================================
    // STATE_TURN_TO_TARGET: 目標方位へ旋回状態
    // ========================================
    case STATE_TURN_TO_TARGET: {
      // 最初の100msは停止（旋回開始前の安定化）
      if (millis() - robot_state.state_start_time < 100) {
        motor_ctrl.stop();
        break;
      }
      
      // PI制御で目標方位への制御入力を計算
      float u = turnTo(TARGET_HEADING);
      
      // 方位角誤差を計算
      float heading_error = TARGET_HEADING - compass_state.current_heading;
      
      // 誤差を-180〜180度の範囲に正規化
      while (heading_error < -180) heading_error += 360;
      while (heading_error > 180) heading_error -= 360;
      
      // 誤差に応じて速度係数を調整
      // 誤差が大きいほど速く旋回、小さいほどゆっくり旋回
      float speed_factor;
      if (abs(heading_error) > 90) speed_factor = 1.0;       // 90度超: 100%速度
      else if (abs(heading_error) > 45) speed_factor = 0.9;  // 45〜90度: 90%速度
      else if (abs(heading_error) > 15) speed_factor = 0.85; // 15〜45度: 85%速度
      else speed_factor = 0.8;                               // 15度未満: 80%速度
      
      // 制御入力が小さい場合は停止（旋回完了判定）
      if (abs(u) < 2) {
        motor_ctrl.stop();
      } else {
        // 左右のモーター速度を計算
        int left = u * speed_factor;
        int right = -u * speed_factor;
        
        // 速度を制限
        left = constrain(left, -130, 130);
        right = constrain(right, -130, 130);
        
        motor_ctrl.setSpeeds(left, right);
      }
      
      // 誤差が10度未満になったら旋回完了
      if (abs(heading_error) < 10.0) {
        motor_ctrl.stop();
        robot_state.mode = STATE_WAIT_AFTER_TURN;
        robot_state.state_start_time = millis();
        pi_ctrl.reset();
      }
      
      // タイムアウト（8秒）したら強制的に次の状態へ
      if (millis() - robot_state.state_start_time > 8000) {
        motor_ctrl.stop();
        robot_state.mode = STATE_WAIT_AFTER_TURN;
        robot_state.state_start_time = millis();
        pi_ctrl.reset();
      }
      break;
    }

    // ========================================
    // STATE_WAIT_AFTER_TURN: 旋回後の待機状態
    // ========================================
    case STATE_WAIT_AFTER_TURN:
      motor_ctrl.stop();
      
      // 500ms待機後、脱出モードへ
      if (millis() - robot_state.state_start_time >= 500) {
        robot_state.mode = STATE_ESCAPE;
        pi_ctrl.reset();
      }
      break;

    // ========================================
    // STATE_ESCAPE: 脱出状態（物体を運搬中）
    // ========================================
    case STATE_ESCAPE: {
      // 黒線検知 → 回避
      if (color_sensor.current_color == COLOR_BLACK) {
        robot_state.mode = STATE_AVOID;
        robot_state.state_start_time = millis();
        break;
      }
      
      // 白→赤 or 白→青 の検知 → 自陣到着
      // 前回の色が白で、現在の色が赤または青なら自陣ゾーンに到着
      if (color_sensor.previous_color == COLOR_WHITE && 
          (color_sensor.current_color == COLOR_RED || 
           color_sensor.current_color == COLOR_BLUE)) {
        motor_ctrl.stop();
        
        // デバッグ情報を表示
        Serial.print(F("Home zone reached: "));
        Serial.println(color_sensor.current_color == COLOR_RED ? F("RED") : F("BLUE"));
        
        // 預け入れモードへ遷移
        robot_state.mode = STATE_DEPOSIT;
        robot_state.state_start_time = millis();
        
        // 運搬カウントを増やす
        robot_state.cups_delivered++;
        Serial.print(F("Cups delivered: "));
        Serial.println(robot_state.cups_delivered);
        break;
      }
      
      // PI制御で方位を維持しながら前進
      float control_u = turnTo(TARGET_HEADING);
      
      // 制御入力が小さい場合は0にする（モーターの遊び対策）
      if (abs(control_u) < 3) {
        control_u = 0;
      }
      
      // 左右のモーター速度を計算
      // 基本速度(MOTOR_ESCAPE) + 制御入力 * 0.3
      int left = MOTOR_ESCAPE + control_u * 0.3;
      int right = MOTOR_ESCAPE - control_u * 0.3;
      
      // 速度を制限
      left = constrain(left, -200, 200);
      right = constrain(right, -200, 200);
      
      motor_ctrl.setSpeeds(left, right);
      break;
    }

// ========================================
// STATE_DEPOSIT: 預け入れ動作状態（1秒後退 + 半回転 + 3秒前進）
// ========================================
case STATE_DEPOSIT:
  if (millis() - robot_state.state_start_time < 1000) {
    // 最初の1秒間：後退
    motor_ctrl.setSpeeds(MOTOR_REVERSE, MOTOR_REVERSE);
  } else if (millis() - robot_state.state_start_time < 2500) {
    // 次の1.5秒間：半回転（180度）
    // 左モーター正転、右モーター逆転で時計回り
    motor_ctrl.setSpeeds(MOTOR_ROTATE, -MOTOR_ROTATE);
  } else if (millis() - robot_state.state_start_time < 5500) {
    // 次の3秒間：前進
    motor_ctrl.setSpeeds(MOTOR_FORWARD, MOTOR_FORWARD);
  } else {
    // 完了したら探索モードへ
    motor_ctrl.stop();
    robot_state.mode = STATE_SEARCH;
    robot_state.search_start_time = millis();
    robot_state.search_rotation_count = 0;
    robot_state.object_detected_in_search = false;
    Serial.println(F("Deposit complete, searching for next cup"));
  }
  break;

    // ========================================
    // STATE_CHECK_ZONE: ゾーン確認状態（削除 - 不要になった）
    // ========================================
    case STATE_CHECK_ZONE:
      // この状態は使用しないが、念のため探索に戻す
      robot_state.mode = STATE_SEARCH;
      robot_state.search_start_time = millis();
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
      break;

// ========================================
// STATE_AVOID: 回避状態（黒線を避ける）
// ========================================
case STATE_AVOID:
  if (millis() - robot_state.state_start_time < 1000) {
    // 最初の1000ms：後退
    motor_ctrl.setSpeeds(MOTOR_REVERSE, MOTOR_REVERSE);
  } else if (millis() - robot_state.state_start_time < 2500) {
    // 次の2500ms：反時計回りに回転
    // 左モーター逆転、右モーター正転
    motor_ctrl.setSpeeds(-MOTOR_AVOID_ROT, MOTOR_AVOID_ROT);
  } else if (millis() - robot_state.state_start_time < 4000) {
    // 次の2000ms（2秒）：前進
    motor_ctrl.setSpeeds(MOTOR_FORWARD, MOTOR_FORWARD);
  } else {
    // 回避完了後、探索モードへ遷移
    motor_ctrl.stop();
    robot_state.mode = STATE_SEARCH;
    robot_state.search_start_time = millis();
    robot_state.search_rotation_count = 0;
    robot_state.object_detected_in_search = false;
    
    // PI制御をリセット
    pi_ctrl.reset();
  }
  break;

    // ========================================
    // STATE_STOP: 停止状態
    // ========================================
    case STATE_STOP:
      motor_ctrl.stop();
      break;
  }
}