#include "definitions.h"

// ============================================
// モード名取得
// ============================================
const char* getModeName(int mode) {
  switch (mode) {
    case STATE_INIT:            return "INIT";
    case STATE_SEARCH:          return "SEARCH";
    case STATE_CHECK_STATIC:    return "CHECK_STATIC";
    case STATE_APPROACH:        return "APPROACH";
    case STATE_TURN_TO_TARGET:  return "TURN_TO_TARGET";
    case STATE_WAIT_AFTER_TURN: return "WAIT_AFTER_TURN";
    case STATE_ESCAPE:          return "ESCAPE";
    case STATE_AVOID:           return "AVOID";
    case STATE_STOP:            return "STOP";
    case STATE_MOVE:            return "MOVE";
    default:                    return "UNKNOWN";
  }
}

// ============================================
// モード変更通知
// ============================================
void printModeChange() {
  if (robot_state.mode != robot_state.previous_mode) {
    Serial.print(">>> MODE CHANGE: ");
    Serial.print(getModeName(robot_state.previous_mode));
    Serial.print(" -> ");
    Serial.println(getModeName(robot_state.mode));
    robot_state.previous_mode = robot_state.mode;
  }
}

// ============================================
// ステータス表示（500msごと）
// ============================================
void printStatus() {
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastPrintTime >= 500) {
    int dist = ultrasonic.getDistance();
    
    Serial.print("[");
    Serial.print(getModeName(robot_state.mode));
    Serial.print("] ");
    
    // 距離情報
    Serial.print("Dist:");
    Serial.print(dist);
    Serial.print("cm ");
    
    // 色情報
    Serial.print("Color:");
    switch(color_sensor.current_color) {
      case COLOR_WHITE: Serial.print("WHITE "); break;
      case COLOR_BLACK: Serial.print("BLACK "); break;
      case COLOR_RED:   Serial.print("RED ");   break;
      case COLOR_BLUE:  Serial.print("BLUE ");  break;
      default:          Serial.print("OTHER "); break;
    }
    
    // 方位情報（必要な状態のみ）
    if (robot_state.mode == STATE_TURN_TO_TARGET || 
        robot_state.mode == STATE_WAIT_AFTER_TURN || 
        robot_state.mode == STATE_ESCAPE) {
      Serial.print("Heading:");
      Serial.print(compass_state.current_heading, 1);
      Serial.print("° Target:");
      Serial.print(TARGET_HEADING, 1);
      Serial.print("° Error:");
      
      float error = TARGET_HEADING - compass_state.current_heading;
      while (error < -180) error += 360;
      while (error > 180) error -= 360;
      Serial.print(error, 1);
      Serial.print("° ");
    }
    
    // モーター速度
    Serial.print("Motor[L:");
    Serial.print(motor_ctrl.left_speed);
    Serial.print(" R:");
    Serial.print(motor_ctrl.right_speed);
    Serial.print("]");
    
    Serial.println();
    lastPrintTime = currentTime;
  }
}

// ============================================
// メインタスク（状態遷移）
// ============================================
void task() {
  robot_state.updateTime();
  printModeChange();
  
  int dist = ultrasonic.getDistance();
  
  switch (robot_state.mode) {
    // ========================================
    // 初期状態
    // ========================================
    case STATE_INIT:
      robot_state.mode = STATE_SEARCH;
      robot_state.search_start_time = millis();
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
      pi_ctrl.reset();
      break;

    // ========================================
    // 探索状態（回転して物体を探す）
    // ========================================
    case STATE_SEARCH: {
      // 物体検出判定
      if (dist > 0 && dist < 30) {
        if (!robot_state.object_detected_in_search) {
          robot_state.object_detected_in_search = true;
          robot_state.search_rotation_count = 0;
        }
        robot_state.search_rotation_count++;
        
        // 3回連続検出で確定
        if (robot_state.search_rotation_count >= 3) {
          motor_ctrl.stop();
          delay(100);
          robot_state.mode = STATE_CHECK_STATIC;
          robot_state.search_rotation_count = 0;
          robot_state.object_detected_in_search = false;
          break;
        }
      } else {
        // 物体が見えなくなったらリセット
        if (robot_state.object_detected_in_search && robot_state.search_rotation_count < 3) {
          robot_state.search_rotation_count = 0;
          robot_state.object_detected_in_search = false;
        }
      }
      
      // 回転して探索
      motor_ctrl.setSpeeds(-MotorSpeeds::ROTATE, MotorSpeeds::ROTATE);
      
      // 5秒経過で移動モードへ
      if (millis() - robot_state.search_start_time > 5000) {
        motor_ctrl.stop();
        robot_state.mode = STATE_MOVE;
        robot_state.state_start_time = millis();
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      }
      break;
    }

    // ========================================
    // 移動状態（場所を変える）
    // ========================================
    case STATE_MOVE:
      motor_ctrl.setSpeeds(MotorSpeeds::MOVE, MotorSpeeds::MOVE);
      
      if (color_sensor.current_color == COLOR_BLACK) {
        robot_state.mode = STATE_AVOID;
        robot_state.state_start_time = millis();
      } else if (dist > 0 && dist < 30) {
        motor_ctrl.stop();
        robot_state.mode = STATE_CHECK_STATIC;
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      } else if (millis() - robot_state.state_start_time > 2000) {
        // 2秒前進後、再探索
        motor_ctrl.stop();
        robot_state.mode = STATE_SEARCH;
        robot_state.search_start_time = millis();
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      }
      break;

    // ========================================
    // 静止判定状態
    // ========================================
    case STATE_CHECK_STATIC:
      if (ultrasonic.isObjectStatic()) {
        robot_state.mode = STATE_APPROACH;
      } else {
        robot_state.mode = STATE_SEARCH;
        robot_state.search_start_time = millis();
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      }
      break;

    // ========================================
    // 接近状態（カップに直進）
    // ========================================
    case STATE_APPROACH:
      if (color_sensor.current_color == COLOR_BLACK) {
        robot_state.mode = STATE_AVOID;
        robot_state.state_start_time = millis();
        break;
      }
      
      motor_ctrl.setSpeeds(MotorSpeeds::FORWARD, MotorSpeeds::FORWARD);
      
      if (dist < 7) {
        robot_state.mode = STATE_TURN_TO_TARGET;
        robot_state.state_start_time = millis();
        pi_ctrl.reset();
      }
      break;

    // ========================================
    // 旋回状態（目標方位に向く）
    // ========================================
    case STATE_TURN_TO_TARGET: {
      // 回転開始前の待機時間を短縮（カップを安定させる）
      if (millis() - robot_state.state_start_time < 100) {
        motor_ctrl.stop();
        break;
      }
      
      float u = turnTo(TARGET_HEADING);
      
      // 角度誤差を計算
      float heading_error = TARGET_HEADING - compass_state.current_heading;
      while (heading_error < -180) heading_error += 360;
      while (heading_error > 180) heading_error -= 360;
      
      // 段階的な速度制御（速度を上げて時間短縮）
      float speed_factor;
      if (abs(heading_error) > 90) {
        speed_factor = 1.0;      // 大角度：100%
      } else if (abs(heading_error) > 45) {
        speed_factor = 0.9;      // 中角度：90%
      } else if (abs(heading_error) > 15) {
        speed_factor = 0.85;     // 小角度：85%
      } else {
        speed_factor = 0.8;      // 微調整：80%
      }
      
      // 制御入力にスピードファクターを適用
      if (abs(u) < 2) {
        motor_ctrl.stop();
      } else {
        int left = u * speed_factor;
        int right = -u * speed_factor;
        
        // 最大速度を少し上げる（カップが外れない範囲で）
        left = constrain(left, -130, 130);
        right = constrain(right, -130, 130);
        motor_ctrl.setSpeeds(left, right);
      }
      
      // 目標方位の許容誤差を少し緩く（10度以内）
      if (abs(heading_error) < 10.0) {
        motor_ctrl.stop();
        robot_state.mode = STATE_WAIT_AFTER_TURN;
        robot_state.state_start_time = millis();
        pi_ctrl.reset();
      }
      
      // タイムアウト（8秒に短縮）
      if (millis() - robot_state.state_start_time > 8000) {
        motor_ctrl.stop();
        robot_state.mode = STATE_WAIT_AFTER_TURN;
        robot_state.state_start_time = millis();
        pi_ctrl.reset();
      }
      break;
    }

    // ========================================
    // 旋回後待機状態（カップを安定させる）
    // ========================================
    case STATE_WAIT_AFTER_TURN:
      motor_ctrl.stop();
      
      if (millis() - robot_state.state_start_time >= 500) {  // 0.5秒待機
        robot_state.mode = STATE_ESCAPE;
        pi_ctrl.reset();
      }
      break;

    // ========================================
    // 運搬状態（方位制御しながら前進）
    // ========================================
    case STATE_ESCAPE: {
      if (color_sensor.current_color == COLOR_BLACK) {
        robot_state.mode = STATE_AVOID;
        robot_state.state_start_time = millis();
        break;
      }
      
      // PI制御で方位を維持しながら前進
      float control_u = turnTo(TARGET_HEADING);
      
      // 不感帯を設けて振動を防止
      if (abs(control_u) < 3) {
        control_u = 0;
      }
      
      // 基本速度に制御入力を加算
      int left = MotorSpeeds::ESCAPE + control_u * 0.3;
      int right = MotorSpeeds::ESCAPE - control_u * 0.3;
      
      // モーター速度の制限
      left = constrain(left, -200, 200);
      right = constrain(right, -200, 200);
      motor_ctrl.setSpeeds(left, right);
      
      // 黒から白に戻ったら停止
      if (color_sensor.previous_color == COLOR_BLACK && color_sensor.current_color == COLOR_WHITE) {
        motor_ctrl.stop();
        robot_state.mode = STATE_STOP;
      }
      break;
    }

    // ========================================
    // 回避状態（黒線を回避）
    // ========================================
    case STATE_AVOID:
      if (millis() - robot_state.state_start_time < 300) {
        // 0.3秒後退
        motor_ctrl.setSpeeds(MotorSpeeds::REVERSE, MotorSpeeds::REVERSE);
      } else if (millis() - robot_state.state_start_time < 700) {
        // 0.4秒回転
        motor_ctrl.setSpeeds(-MotorSpeeds::AVOID_ROT, MotorSpeeds::AVOID_ROT);
      } else {
        // 次の状態へ
        robot_state.mode = (dist < 5 ? STATE_TURN_TO_TARGET : STATE_SEARCH);
        if (robot_state.mode == STATE_SEARCH) {
          robot_state.search_start_time = millis();
          robot_state.search_rotation_count = 0;
          robot_state.object_detected_in_search = false;
        } else {
          robot_state.state_start_time = millis();
        }
        pi_ctrl.reset();
      }
      break;

    // ========================================
    // 停止状態
    // ========================================
    case STATE_STOP:
      motor_ctrl.stop();
      break;
  }
}