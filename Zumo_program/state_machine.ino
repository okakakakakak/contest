#include "definitions.h"

// ============================================
// モード名取得
// ============================================
const char* getModeName(int mode) {
  switch (mode) {
    case INIT:            return "INIT";
    case SEARCH:          return "SEARCH";
    case CHECK_STATIC:    return "CHECK_STATIC";
    case APPROACH:        return "APPROACH";
    case TURN_TO_TARGET:  return "TURN_TO_TARGET";
    case WAIT_AFTER_TURN: return "WAIT_AFTER_TURN";
    case ESCAPE:          return "ESCAPE";
    case AVOID:           return "AVOID";
    case STOP:            return "STOP";
    case MOVE:            return "MOVE";
    case GOAL:            return "GOAL";
    default:              return "UNKNOWN";
  }
}

// ============================================
// モード変更通知
// ============================================
void printModeChange() {
  if (mode != prevMode) {
    Serial.print(">>> MODE CHANGE: ");
    Serial.print(getModeName(prevMode));
    Serial.print(" -> ");
    Serial.println(getModeName(mode));
    prevMode = mode;
  }
}

// ============================================
// ステータス表示（500msごと）
// ============================================
void printStatus() {
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastPrintTime >= 500) {
    Serial.print("[");
    Serial.print(getModeName(mode));
    Serial.print("] ");
    
    // 距離情報
    Serial.print("Dist:");
    Serial.print(dist);
    Serial.print("cm ");
    
    // 色情報
    Serial.print("Color:");
    switch(color) {
      case WHITE: Serial.print("WHITE "); break;
      case BLACK: Serial.print("BLACK "); break;
      case RED:   Serial.print("RED ");   break;
      case BLUE:  Serial.print("BLUE ");  break;
      default:    Serial.print("OTHER "); break;
    }
    
    // 方位情報（必要な状態のみ）
    if (mode == TURN_TO_TARGET || mode == WAIT_AFTER_TURN || mode == ESCAPE) {
      Serial.print("Heading:");
      Serial.print(heading_G, 1);
      Serial.print("° Target:");
      Serial.print(TARGET_HEADING, 1);
      Serial.print("° Error:");
      
      float error = TARGET_HEADING - heading_G;
      while (error < -180) error += 360;
      while (error > 180) error -= 360;
      Serial.print(error, 1);
      Serial.print("° ");
    }
    
    // モーター速度
    Serial.print("Motor[L:");
    Serial.print(motorL);
    Serial.print(" R:");
    Serial.print(motorR);
    Serial.print("]");
    
    Serial.println();
    lastPrintTime = currentTime;
  }

   // モード送信（Processing用）
  Serial.print("MODE:");
  Serial.println(getModeName(mode));

  // 距離送信
  Serial.print("DIST:");
  Serial.println(dist);

  // 色識別送信
  Serial.print("COLOR:");
  switch(color) {
    case WHITE: Serial.println("WHITE"); break;
    case BLACK: Serial.println("BLACK"); break;
    case RED:   Serial.println("RED");   break;
    case BLUE:  Serial.println("BLUE");  break;
    default:    Serial.println("OTHER"); break;
  }

  // 方位角送信（必要なモードのみ）
  if (mode == TURN_TO_TARGET || mode == WAIT_AFTER_TURN || mode == ESCAPE) {
    Serial.print("HEADING:");
    Serial.println(heading_G, 1);
  }

  // モーター速度送信
  Serial.print("MOTOR:");
  Serial.print(motorL);
  Serial.print(",");
  Serial.println(motorR);
}

// ============================================
// メインタスク（状態遷移）
// ============================================
void task() {
  timeNow_G = millis();
  printModeChange();
  
  switch (mode) {
    // ========================================
    // 初期状態
    // ========================================
    case INIT:
      mode = SEARCH;
      searchStartTime = millis();
      searchRotationCount = 0;
      objectDetectedInSearch = false;
      sum_e = 0;
      break;

    // ========================================
    // 探索状態（回転して物体を探す）
    // ========================================
    case SEARCH: {
      // 物体検出判定
      if (dist > 0 && dist < 30) {
        if (!objectDetectedInSearch) {
          objectDetectedInSearch = true;
          searchRotationCount = 0;
        }
        searchRotationCount++;
        
        // 3回連続検出で確定
        if (searchRotationCount >= 3) {
          motorL = motorR = SPEED_STOP;
          motors.setSpeeds(motorL, motorR);
          delay(100);
          mode = CHECK_STATIC;
          searchRotationCount = 0;
          objectDetectedInSearch = false;
          break;
        }
      } else {
        // 物体が見えなくなったらリセット
        if (objectDetectedInSearch && searchRotationCount < 3) {
          searchRotationCount = 0;
          objectDetectedInSearch = false;
        }
      }
      
      // 回転して探索
      motorL = -SPEED_ROTATE;
      motorR = SPEED_ROTATE;
      
      // 5秒経過で移動モードへ
      if (millis() - searchStartTime > 5000) {
        motorL = motorR = SPEED_STOP;
        mode = MOVE;
        start_time = millis();
        searchRotationCount = 0;
        objectDetectedInSearch = false;
      }
      break;
    }

    // ========================================
    // 移動状態（場所を変える）
    // ========================================
    case MOVE:
      motorL = motorR = SPEED_MOVE;
      
      if (color == BLACK) {
        mode = AVOID;
        start_time = millis();
      } else if (dist > 0 && dist < 30) {
        motorL = motorR = SPEED_STOP;
        mode = CHECK_STATIC;
        searchRotationCount = 0;
        objectDetectedInSearch = false;
      } else if (millis() - start_time > 2000) {
        // 2秒前進後、再探索
        motorL = motorR = SPEED_STOP;
        mode = SEARCH;
        searchStartTime = millis();
        searchRotationCount = 0;
        objectDetectedInSearch = false;
      }
      break;

    // ========================================
    // 静止判定状態
    // ========================================
    case CHECK_STATIC:
      if (isCupStatic()) {
        mode = APPROACH;
      } else {
        mode = SEARCH;
        searchStartTime = millis();
        searchRotationCount = 0;
        objectDetectedInSearch = false;
      }
      break;

    // ========================================
    // 接近状態（カップに直進）
    // ========================================
    case APPROACH:
      if (color == BLACK) {
        mode = AVOID;
        start_time = millis();
        break;
      }
      
      motorL = motorR = SPEED_FORWARD;
      
      if (dist < 7) {
        mode = TURN_TO_TARGET;
        start_time = millis();
        sum_e = 0;
      }
      break;

    // ========================================
    // 旋回状態（目標方位に向く）
    // ========================================
    case TURN_TO_TARGET: {
      // 回転開始前の待機時間を短縮（カップを安定させる）
      if (millis() - start_time < 100) {
        motorL = motorR = SPEED_STOP;
        motors.setSpeeds(motorL, motorR);
        break;
      }
      
      updateHeading();
      float u = turnTo(TARGET_HEADING);
      
      // 角度誤差を計算
      float heading_error = TARGET_HEADING - heading_G;
      while (heading_error < -180) heading_error += 360;
      while (heading_error > 180) heading_error -= 360;
      
      // 段階的な速度制御（速度を上げて時間短縮）
      float speed_factor;
      if (abs(heading_error) > 90) {
        speed_factor = 1.0;      // 大角度：100%
      } else if (abs(heading_error) > 45) {
        speed_factor = 0.9;      // 中角度：90%
      } else if (abs(heading_error) > 15) {
        speed_factor = 0.85;      // 小角度：85%
      } else {
        speed_factor = 0.8;      // 微調整：80%
      }
      
      // 制御入力にスピードファクターを適用
      if (abs(u) < 2) {
        motorL = motorR = SPEED_STOP;
      } else {
        motorL = u * speed_factor;
        motorR = -u * speed_factor;
        
        // 最大速度を少し上げる（カップが外れない範囲で）
        motorL = constrain(motorL, -130, 130);
        motorR = constrain(motorR, -130, 130);
      }
      
      // 目標方位の許容誤差を少し緩く（10度以内）
      if (abs(heading_error) < 10.0) {
        motorL = motorR = SPEED_STOP;
        motors.setSpeeds(motorL, motorR);
        mode = WAIT_AFTER_TURN;
        start_time = millis();
        sum_e = 0;
      }
      
      // タイムアウト（8秒に短縮）
      if (millis() - start_time > 8000) {
        motorL = motorR = SPEED_STOP;
        motors.setSpeeds(motorL, motorR);
        mode = WAIT_AFTER_TURN;
        start_time = millis();
        sum_e = 0;
      }
      break;
    }

    // ========================================
    // 旋回後待機状態（カップを安定させる）
    // ========================================
    case WAIT_AFTER_TURN:
      motorL = motorR = SPEED_STOP;
      motors.setSpeeds(motorL, motorR);
      
      if (millis() - start_time >= 500) {  // 0.5秒待機
        mode = ESCAPE;
        sum_e = 0;
      }
      break;

    // ========================================
    // 運搬状態（方位制御しながら前進）
    // ========================================
    case ESCAPE: {
      if (color == BLACK) {
        mode = AVOID;
        start_time = millis();
        break;
      }
      
      // PI制御で方位を維持しながら前進
      float control_u = turnTo(TARGET_HEADING);
      
      // 不感帯を設けて振動を防止
      if (abs(control_u) < 3) {
        control_u = 0;
      }
      
      // 基本速度に制御入力を加算
      motorL = SPEED_ESCAPE + control_u * 0.3;
      motorR = SPEED_ESCAPE - control_u * 0.3;
      
      // モーター速度の制限
      motorL = constrain(motorL, -200, 200);
      motorR = constrain(motorR, -200, 200);
      
      // 黒から白に戻ったら停止
      if (prevColor == BLACK && color == WHITE) {
        motorL = motorR = SPEED_STOP;
        motors.setSpeeds(motorL, motorR);
        mode = STOP;
      }
      break;
    }

    // ========================================
    // 回避状態（黒線を回避）
    // ========================================
    case AVOID:
      if (millis() - start_time < 300) {
        // 0.3秒後退
        motorL = motorR = SPEED_REVERSE;
      } else if (millis() - start_time < 700) {
        // 0.4秒回転
        motorL = -SPEED_AVOID_ROT;
        motorR = SPEED_AVOID_ROT;
      } else {
        // 次の状態へ
        mode = (dist < 5 ? TURN_TO_TARGET : SEARCH);
        if (mode == SEARCH) {
          searchStartTime = millis();
          searchRotationCount = 0;
          objectDetectedInSearch = false;
        } else {
          start_time = millis();
        }
        sum_e = 0;
      }
      break;

    // ========================================
    // ゴールラインまで運搬
    // ========================================
    case GOAL:
      if (millis() - start_time < 1000) {
        // 1秒後退
        motorL = motorR = SPEED_REVERSE;
      } else if (millis() - start_time < 1800) {
        // 0.8秒回転
        motorL = -SPEED_AVOID_ROT;
        motorR = SPEED_AVOID_ROT;
      } else {
        mode = MOVE;
        start_time = millis();
        searchRotationCount = 0;
        objectDetectedInSearch = false;
      }
      break;

    // ========================================
    // 停止状態
    // ========================================
    case STOP:
      motorL = motorR = SPEED_STOP;
      motors.setSpeeds(motorL, motorR);
      break;
  }
  
  timePrev_G = timeNow_G;

}
