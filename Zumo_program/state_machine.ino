#include "definitions.h"

// ============================================
// モード名取得（PROGMEM使用でRAM節約）
// ============================================
const char str_init[] PROGMEM = "INIT";
const char str_search[] PROGMEM = "SEARCH";
const char str_check[] PROGMEM = "CHECK_STATIC";
const char str_approach[] PROGMEM = "APPROACH";
const char str_turn[] PROGMEM = "TURN_TO_TARGET";
const char str_wait[] PROGMEM = "WAIT_AFTER_TURN";
const char str_escape[] PROGMEM = "TRANSPORT";
const char str_avoid[] PROGMEM = "AVOID";
const char str_stop[] PROGMEM = "STOP";
const char str_move[] PROGMEM = "MOVE";
const char str_check_zone[] PROGMEM = "CHECK_ZONE";
const char str_deposit[] PROGMEM = "DEPOSIT";
const char str_unknown[] PROGMEM = "UNKNOWN";

const char* const mode_names[] PROGMEM = {
  str_init, str_search, str_check, str_approach, str_turn,
  str_wait, str_escape, str_avoid, str_stop, str_move,
  str_check_zone, str_deposit
};

void printModeName(byte mode) {
  if (mode < 12) {
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
void printModeChange() {
  if (robot_state.mode != robot_state.previous_mode) {
    Serial.print(F(">>> "));
    printModeName(robot_state.previous_mode);
    Serial.print(F(" -> "));
    printModeName(robot_state.mode);
    Serial.println();
    robot_state.previous_mode = robot_state.mode;
  }
}

// ============================================
// ステータス表示（500msごと、簡略化していない最新版）
// ============================================
void printStatus() {
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastPrintTime >= 500) {
    int dist = ultrasonic.getDistance();
    int colorCode = color_sensor.current_color;
    float heading = compass_state.current_heading;
    int motorL = motor_ctrl.left_speed;
    int motorR = motor_ctrl.right_speed;

    // モード名
    Serial.print("MODE:");
    printModeName(robot_state.mode);
    Serial.println();

    // 距離
    Serial.print("DIST:");
    Serial.println(dist);

    // 色（数値 → 文字列変換）
    Serial.print("COLOR:");
    switch (colorCode) {
      case COLOR_WHITE: Serial.println("WHITE"); break;
      case COLOR_RED:   Serial.println("RED");   break;
      case COLOR_BLACK: Serial.println("BLACK"); break;
      case COLOR_BLUE:  Serial.println("BLUE");  break;
      default:          Serial.println("OTHER"); break;
    }

    // 方位（必要なモードのみ）
    if (robot_state.mode == STATE_TURN_TO_TARGET || robot_state.mode == STATE_TRANSPORT) {
      Serial.print("HEADING:");
      Serial.println(heading, 0);  // 小数点なしで送信
    }

    // モーター速度
    Serial.print("MOTOR:");
    Serial.print(motorL);
    Serial.print(",");
    Serial.println(motorR);

    //追加：加速度
    Serial.print("ACCEL:");


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
    case STATE_INIT:
      robot_state.mode = STATE_SEARCH;
      robot_state.search_start_time = millis();
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
      pi_ctrl.reset();
      break;

    case STATE_SEARCH: {
      if (dist > 0 && dist < 30) {
        if (!robot_state.object_detected_in_search) {
          robot_state.object_detected_in_search = true;
          robot_state.search_rotation_count = 0;
        }
        robot_state.search_rotation_count++;
        
        if (robot_state.search_rotation_count >= 3) {
          motor_ctrl.stop();
          delay(100);
          robot_state.mode = STATE_CHECK_STATIC;
          robot_state.search_rotation_count = 0;
          robot_state.object_detected_in_search = false;
          break;
        }
      } else {
        if (robot_state.object_detected_in_search && robot_state.search_rotation_count < 3) {
          robot_state.search_rotation_count = 0;
          robot_state.object_detected_in_search = false;
        }
      }
      
      motor_ctrl.setSpeeds(-MOTOR_ROTATE, MOTOR_ROTATE);
      
      if (millis() - robot_state.search_start_time > 5000) {
        motor_ctrl.stop();
        robot_state.mode = STATE_MOVE;
        robot_state.state_start_time = millis();
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      }
      break;
    }

    case STATE_MOVE:
      motor_ctrl.setSpeeds(MOTOR_MOVE, MOTOR_MOVE);
      
      if (color_sensor.current_color == COLOR_BLACK) {
        robot_state.mode = STATE_AVOID;
        robot_state.state_start_time = millis();
      } else if (dist > 0 && dist < 30) {
        motor_ctrl.stop();
        robot_state.mode = STATE_CHECK_STATIC;
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      } else if (millis() - robot_state.state_start_time > 2000) {
        motor_ctrl.stop();
        robot_state.mode = STATE_SEARCH;
        robot_state.search_start_time = millis();
        robot_state.search_rotation_count = 0;
        robot_state.object_detected_in_search = false;
      }
      break;

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

    case STATE_APPROACH:
      if (color_sensor.current_color == COLOR_BLACK) {
        robot_state.mode = STATE_AVOID;
        robot_state.state_start_time = millis();
        break;
      }
      
      motor_ctrl.setSpeeds(MOTOR_FORWARD, MOTOR_FORWARD);
      
      if (dist < 7) {
        robot_state.mode = STATE_TURN_TO_TARGET;
        robot_state.state_start_time = millis();
        pi_ctrl.reset();
      }
      break;

    case STATE_TURN_TO_TARGET: {
      if (millis() - robot_state.state_start_time < 100) {
        motor_ctrl.stop();
        break;
      }
      
      float u = turnTo(TARGET_HEADING);
      
      float heading_error = TARGET_HEADING - compass_state.current_heading;
      while (heading_error < -180) heading_error += 360;
      while (heading_error > 180) heading_error -= 360;
      
      float speed_factor;
      if (abs(heading_error) > 90) speed_factor = 1.0;
      else if (abs(heading_error) > 45) speed_factor = 0.9;
      else if (abs(heading_error) > 15) speed_factor = 0.85;
      else speed_factor = 0.8;
      
      if (abs(u) < 2) {
        motor_ctrl.stop();
      } else {
        int left = u * speed_factor;
        int right = -u * speed_factor;
        left = constrain(left, -130, 130);
        right = constrain(right, -130, 130);
        motor_ctrl.setSpeeds(left, right);
      }
      
      if (abs(heading_error) < 10.0) {
        motor_ctrl.stop();
        robot_state.mode = STATE_WAIT_AFTER_TURN;
        robot_state.state_start_time = millis();
        pi_ctrl.reset();
      }
      
      if (millis() - robot_state.state_start_time > 8000) {
        motor_ctrl.stop();
        robot_state.mode = STATE_WAIT_AFTER_TURN;
        robot_state.state_start_time = millis();
        pi_ctrl.reset();
      }
      break;
    }

    case STATE_WAIT_AFTER_TURN:
      motor_ctrl.stop();
      
      if (millis() - robot_state.state_start_time >= 500) {
        robot_state.mode = STATE_TRANSPORT;
        pi_ctrl.reset();
      }
      break;

    // ========================================
    // 運搬状態（方位制御しながら前進）
    // ========================================
    case STATE_TRANSPORT: {
      // 黒線検知 → 回避
      if (color_sensor.current_color == COLOR_BLACK) {
        robot_state.mode = STATE_AVOID;
        robot_state.state_start_time = millis();
        break;
      }
      
      // 白→赤 or 白→青 の検知 → 自陣到着
      if (color_sensor.previous_color == COLOR_WHITE && 
          (color_sensor.current_color == COLOR_RED || 
           color_sensor.current_color == COLOR_BLUE)) {
        motor_ctrl.stop();
        Serial.print(F("Home zone reached: "));
        Serial.println(color_sensor.current_color == COLOR_RED ? F("RED") : F("BLUE"));
        robot_state.mode = STATE_DEPOSIT;
        robot_state.state_start_time = millis();
        robot_state.cups_delivered++;
        Serial.print(F("Cups delivered: "));
        Serial.println(robot_state.cups_delivered);
        break;
      }
      
      // PI制御で方位を維持しながら前進
      float control_u = turnTo(TARGET_HEADING);
      
      if (abs(control_u) < 3) {
        control_u = 0;
      }
      
      int left = MOTOR_TRANSPORT + control_u * 0.3;
      int right = MOTOR_TRANSPORT - control_u * 0.3;
      
      left = constrain(left, -200, 200);
      right = constrain(right, -200, 200);
      motor_ctrl.setSpeeds(left, right);
      break;
    }

    // ========================================
    // 預け入れ動作状態（1秒後退 + 半回転）
    // ========================================
    case STATE_DEPOSIT:
      if (millis() - robot_state.state_start_time < 1000) {
        // 1秒間後退
        motor_ctrl.setSpeeds(MOTOR_REVERSE, MOTOR_REVERSE);
      } else if (millis() - robot_state.state_start_time < 2500) {
        // 1.5秒間半回転（180度）
        motor_ctrl.setSpeeds(MOTOR_ROTATE, -MOTOR_ROTATE);
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
    // ゾーン確認状態（削除 - 不要になった）
    // ========================================
    case STATE_CHECK_ZONE:
      // この状態は使用しないが、念のため探索に戻す
      robot_state.mode = STATE_SEARCH;
      robot_state.search_start_time = millis();
      robot_state.search_rotation_count = 0;
      robot_state.object_detected_in_search = false;
      break;

    case STATE_AVOID:
      if (millis() - robot_state.state_start_time < 300) {
        motor_ctrl.setSpeeds(MOTOR_REVERSE, MOTOR_REVERSE);
      } else if (millis() - robot_state.state_start_time < 700) {
        motor_ctrl.setSpeeds(-MOTOR_AVOID_ROT, MOTOR_AVOID_ROT);
      } else {
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

    case STATE_STOP:
      motor_ctrl.stop();
      break;
  }
}