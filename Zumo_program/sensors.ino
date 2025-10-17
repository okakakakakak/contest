#include "definitions.h"

// ============================================
// 色識別関数
// ============================================
int identify_color(int r, int g, int b) {
  if (200 < r && 200 < g && 200 < b) return WHITE;
  else if (r < 50 && g < 50 && b < 50) return BLACK;
  else if (100 < r && g < 50 && b < 50) return WHITE;
  else if (r < 50 && g < 50 && 70 < b) return WHITE;
  else return OTHER;
}

// ============================================
// 距離測定関数（超音波センサー）
// ============================================
int distance() {
  unsigned long interval;
  int dst;
  
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  interval = pulseIn(echo, HIGH, 5767);
  dst = (0.61 * 25 + 331.5) * interval / 10000 / 2;
  
  if (dst == 0) dst = 100;
  delay(60);
  
  return dst;
}

// ============================================
// 物体静止判定関数
// ============================================
bool isCupStatic() {
  const int checkDuration = 500;
  const int interval = 100;
  const int threshold = 3;
  const int delta = 2;

  motors.setSpeeds(SPEED_STOP, SPEED_STOP);

  int prev = distance();
  int changes = 0;
  unsigned long checkStart = millis();

  while (millis() - checkStart < checkDuration) {
    int now = distance();
    if (abs(now - prev) > delta) {
      changes++;
    }
    prev = now;
    delay(interval);
  }

  return (changes < threshold);
}

// ============================================
// 地磁気センサーキャリブレーション
// ============================================
void calibrationCompass() {
  unsigned long startTime = millis();
  
  Serial.println("[CALIB] Calibration started - rotating for 10 seconds...");
  
  while (millis() - startTime < 10000) {
    compass.read();
    
    // 最小値・最大値の更新
    if (compass.m.x < compass.m_min.x) compass.m_min.x = compass.m.x;
    if (compass.m.x > compass.m_max.x) compass.m_max.x = compass.m.x;
    if (compass.m.y < compass.m_min.y) compass.m_min.y = compass.m.y;
    if (compass.m.y > compass.m_max.y) compass.m_max.y = compass.m.y;
    
    motors.setSpeeds(120, -120);
  }
  
  motors.setSpeeds(SPEED_STOP, SPEED_STOP);
  delay(500);
}

// ============================================
// カラーセンサーキャリブレーション
// ============================================
void calibrationColorSensor() {
  unsigned long timeInit;
  uint16_t r, g, b, clr;
  
  Serial.println("[CALIB] Color sensor calibration started...");
  Serial.println("[CALIB] Move forward for 2 seconds to sample colors");
  
  motors.setSpeeds(60, 60); 

  // 初期化：最小値を大きく、最大値を小さく設定
  r_min = 65535;
  g_min = 65535;
  b_min = 65535;
  r_max = 0;
  g_max = 0;
  b_max = 0;

  timeInit = millis();

  while (millis() - timeInit < 2000) {
    tcs.getRawData(&r, &g, &b, &clr);
  
    // 最小値の更新
    if (r < r_min) r_min = r;
    if (g < g_min) g_min = g;
    if (b < b_min) b_min = b;
    
    // 最大値の更新
    if (r > r_max) r_max = r;
    if (g > g_max) g_max = g;
    if (b > b_max) b_max = b;
    
    delay(50);
  }
  
  motors.setSpeeds(SPEED_STOP, SPEED_STOP);
  
  // キャリブレーション結果を表示
  Serial.println("[CALIB] Color sensor calibration complete!");
  Serial.print("[CALIB] R range: ");
  Serial.print(r_min);
  Serial.print(" to ");
  Serial.println(r_max);
  Serial.print("[CALIB] G range: ");
  Serial.print(g_min);
  Serial.print(" to ");
  Serial.println(g_max);
  Serial.print("[CALIB] B range: ");
  Serial.print(b_min);
  Serial.print(" to ");
  Serial.println(b_max);
  
  // 値が有効かチェック
  if (r_max <= r_min) {
    Serial.println("[WARNING] R calibration may be invalid!");
    r_min = 60;
    r_max = 255;
  }
  if (g_max <= g_min) {
    Serial.println("[WARNING] G calibration may be invalid!");
    g_min = 52;
    g_max = 255;
  }
  if (b_max <= b_min) {
    Serial.println("[WARNING] B calibration may be invalid!");
    b_min = 62;
    b_max = 255;
  }
}

// ============================================
// RGB値取得関数
// ============================================
void getRGB(float& r0, float& g0, float& b0) {
  uint16_t r, g, b, clr;

  tcs.getRawData(&r, &g, &b, &clr);
 
  r0 = map(r, r_min, r_max, 0, 255);
  g0 = map(g, g_min, g_max, 0, 255);
  b0 = map(b, b_min, b_max, 0, 255);

  // 範囲制限
  if (r0 < 0.0) r0 = 0.0;
  if (r0 > 255.0) r0 = 255.0;
  if (g0 < 0.0) g0 = 0.0;
  if (g0 > 255.0) g0 = 255.0;
  if (b0 < 0.0) b0 = 0.0;
  if (b0 > 255.0) b0 = 255.0;
}

// ============================================
// 現在の方位角取得
// ============================================
void updateHeading() {
  compass.read();
  mx = map(compass.m.x, compass.m_min.x, compass.m_max.x, -128, 127);
  my = map(compass.m.y, compass.m_min.y, compass.m_max.y, -128, 127);
  heading_G = atan2(my, mx) * 180.0 / M_PI;
  if (heading_G < 0) heading_G += 360;
}