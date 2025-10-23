#include "definitions.h"

// ============================================
// ColorSensorState メソッド実装
// ============================================
void ColorSensorState::getRGB(float& r, float& g, float& b) {
  uint16_t r_raw, g_raw, b_raw, clr;
  tcs.getRawData(&r_raw, &g_raw, &b_raw, &clr);
 
  r = map(r_raw, r_min, r_max, 0, 255);
  g = map(g_raw, g_min, g_max, 0, 255);
  b = map(b_raw, b_min, b_max, 0, 255);

  r = constrain(r, 0.0, 255.0);
  g = constrain(g, 0.0, 255.0);
  b = constrain(b, 0.0, 255.0);
}

int ColorSensorState::identifyColor(int r, int g, int b) {
  if (200 < r && 200 < g && 200 < b) return COLOR_WHITE;
  else if (r < 50 && g < 50 && b < 50) return COLOR_BLACK;
  else if (100 < r && g < 50 && b < 50) return COLOR_WHITE;
  else if (r < 50 && g < 50 && 70 < b) return COLOR_WHITE;
  else return COLOR_OTHER;
}

void ColorSensorState::calibrate() {
  Serial.println("[CALIB] Color sensor calibration started...");
  Serial.println("[CALIB] Move forward for 2 seconds to sample colors");
  
  motor_ctrl.setSpeeds(60, 60);

  r_min = 65535;
  g_min = 65535;
  b_min = 65535;
  r_max = 0;
  g_max = 0;
  b_max = 0;

  unsigned long start_time = millis();

  while (millis() - start_time < 2000) {
    uint16_t r, g, b, clr;
    tcs.getRawData(&r, &g, &b, &clr);
  
    if (r < r_min) r_min = r;
    if (g < g_min) g_min = g;
    if (b < b_min) b_min = b;
    
    if (r > r_max) r_max = r;
    if (g > g_max) g_max = g;
    if (b > b_max) b_max = b;
    
    delay(50);
  }
  
  motor_ctrl.stop();
  
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
// UltrasonicSensor メソッド実装
// ============================================
void UltrasonicSensor::init() {
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
}

int UltrasonicSensor::getDistance() {
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  
  unsigned long interval = pulseIn(echo_pin, HIGH, 5767);
  int dst = (0.61 * 25 + 331.5) * interval / 10000 / 2;
  
  if (dst == 0) dst = 100;
  delay(60);
  
  return dst;
}

bool UltrasonicSensor::isObjectStatic() {
  const int checkDuration = 500;
  const int interval = 100;
  const int threshold = 3;
  const int delta = 2;

  motor_ctrl.stop();

  int prev = getDistance();
  int changes = 0;
  unsigned long checkStart = millis();

  while (millis() - checkStart < checkDuration) {
    int now = getDistance();
    if (abs(now - prev) > delta) {
      changes++;
    }
    prev = now;
    delay(interval);
  }

  return (changes < threshold);
}

// ============================================
// CompassState メソッド実装
// ============================================
void CompassState::updateHeading(float magnetic_declination) {
  compass.read();
  
  // ハードアイアン補正を適用
  float mx_corrected = (compass.m.x - calib.offset_x) * calib.scale_x;
  float my_corrected = (compass.m.y - calib.offset_y) * calib.scale_y;
  
  // -128〜127にマッピング
  float mx = map(mx_corrected, 
                 compass.m_min.x - calib.offset_x, 
                 compass.m_max.x - calib.offset_x, 
                 -128, 127);
  float my = map(my_corrected,
                 compass.m_min.y - calib.offset_y,
                 compass.m_max.y - calib.offset_y,
                 -128, 127);
  
  // 生の方位角を計算
  float raw_heading = atan2(my, mx) * 180.0 / M_PI;
  if (raw_heading < 0) raw_heading += 360;
  
  // 偏角補正
  raw_heading += magnetic_declination;
  if (raw_heading < 0) raw_heading += 360;
  if (raw_heading >= 360) raw_heading -= 360;
  
  // 移動平均フィルタ
  heading_buffer[heading_index] = raw_heading;
  heading_index = (heading_index + 1) % HEADING_FILTER_SIZE;
  
  float sum = 0;
  for (int i = 0; i < HEADING_FILTER_SIZE; i++) {
    sum += heading_buffer[i];
  }
  current_heading = sum / HEADING_FILTER_SIZE;
}

// ============================================
// コンパスキャリブレーション（グローバル関数として残す）
// ============================================
static void updateMinMax(int x, int y, float& mx_min, float& mx_max, float& my_min, float& my_max) {
  if (x < mx_min) mx_min = x;
  if (x > mx_max) mx_max = x;
  if (y < my_min) my_min = y;
  if (y > my_max) my_max = y;
}

void calibrationCompassAdvanced() {
  float mx_max = -32768, my_max = -32768;
  float mx_min = 32767, my_min = 32767;
  int sampleCount = 0;
  
  Serial.println("[CALIB] Advanced compass calibration - 15 seconds rotation");
  
  unsigned long startTime = millis();
  
  // 時計回り 5秒
  Serial.println("[CALIB] Phase 1/3: Clockwise rotation");
  while (millis() - startTime < 5000) {
    compass_state.compass.read();
    updateMinMax(compass_state.compass.m.x, compass_state.compass.m.y, 
                 mx_min, mx_max, my_min, my_max);
    motor_ctrl.setSpeeds(120, -120);
    delay(50);
    sampleCount++;
  }
  
  // 反時計回り 5秒
  Serial.println("[CALIB] Phase 2/3: Counter-clockwise rotation");
  startTime = millis();
  while (millis() - startTime < 5000) {
    compass_state.compass.read();
    updateMinMax(compass_state.compass.m.x, compass_state.compass.m.y,
                 mx_min, mx_max, my_min, my_max);
    motor_ctrl.setSpeeds(-120, 120);
    delay(50);
    sampleCount++;
  }
  
  // 時計回り 5秒
  Serial.println("[CALIB] Phase 3/3: Clockwise rotation");
  startTime = millis();
  while (millis() - startTime < 5000) {
    compass_state.compass.read();
    updateMinMax(compass_state.compass.m.x, compass_state.compass.m.y,
                 mx_min, mx_max, my_min, my_max);
    motor_ctrl.setSpeeds(120, -120);
    delay(50);
    sampleCount++;
  }
  
  motor_ctrl.stop();
  delay(500);
  
  // ハードアイアン補正を計算
  compass_state.calib.offset_x = (mx_max + mx_min) / 2.0;
  compass_state.calib.offset_y = (my_max + my_min) / 2.0;
  
  // スケール補正を計算
  float range_x = (mx_max - mx_min) / 2.0;
  float range_y = (my_max - my_min) / 2.0;
  float avg_range = (range_x + range_y) / 2.0;
  
  compass_state.calib.scale_x = avg_range / range_x;
  compass_state.calib.scale_y = avg_range / range_y;
  
  // compass.m_min/maxに保存
  compass_state.compass.m_min.x = mx_min;
  compass_state.compass.m_max.x = mx_max;
  compass_state.compass.m_min.y = my_min;
  compass_state.compass.m_max.y = my_max;
  
  // 結果表示
  Serial.print("[CALIB] Total samples: "); 
  Serial.println(sampleCount);
  Serial.print("[CALIB] X range: ");
  Serial.print(mx_min);
  Serial.print(" to ");
  Serial.println(mx_max);
  Serial.print("[CALIB] Y range: ");
  Serial.print(my_min);
  Serial.print(" to ");
  Serial.println(my_max);
  Serial.print("[CALIB] Hard-iron offset X: "); 
  Serial.println(compass_state.calib.offset_x);
  Serial.print("[CALIB] Hard-iron offset Y: "); 
  Serial.println(compass_state.calib.offset_y);
  Serial.print("[CALIB] Scale factor X: "); 
  Serial.println(compass_state.calib.scale_x, 4);
  Serial.print("[CALIB] Scale factor Y: "); 
  Serial.println(compass_state.calib.scale_y, 4);
}