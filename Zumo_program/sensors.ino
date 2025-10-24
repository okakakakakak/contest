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

byte ColorSensorState::identifyColor(int r, int g, int b) {
  // 白（全ての値が高い）
  if (r > 240 && g > 240 && b > 240) return COLOR_WHITE;
  
  // 黒（全ての値が低い）
  if (r < 10 && g < 10 && b < 10) return COLOR_BLACK;
  
  // 赤（Rが高く、G・Bが低い）
  if (r > 100 && g < 80 && b < 80) return COLOR_RED;
  
  // 青（Bが高く、R・Gが低い）
  if (r < 80 && g < 80 && b > 100) return COLOR_BLUE;
  
  // その他
  return COLOR_OTHER;
}

void ColorSensorState::calibrate() {
  Serial.println(F("Calibrating..."));
  
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
  
  // 値が有効かチェック（簡略版）
  if (r_max <= r_min) {
    r_min = 60;
    r_max = 255;
  }
  if (g_max <= g_min) {
    g_min = 52;
    g_max = 255;
  }
  if (b_max <= b_min) {
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
  for (byte i = 0; i < HEADING_FILTER_SIZE; i++) {
    sum += heading_buffer[i];
  }
  current_heading = sum / HEADING_FILTER_SIZE;
}

// ============================================
// 加速度センサーによる傾斜検知
// ============================================
bool isSlopeDetected() {
  static unsigned long lastAccelRead = 0;
  static int current_accel_z = 0;
  
  // 頻繁に読み取ると動作に影響するため、一定間隔で読み取る
  if (millis() - lastAccelRead > ACCEL_READ_INTERVAL) {
    compass_state.compass.readAcc();
    // Z軸の値を読み取り、オフセットを適用
    current_accel_z = compass_state.compass.a.z + ACCEL_Z_OFFSET;
    lastAccelRead = millis();
  }
  
  // Z軸の値が閾値を超えていれば坂道と判定（坂を登る場合、Z軸の値が増加/減少する）
  // 坂道の向きによって符号が変わるため、絶対値で判定します
  if (abs(current_accel_z) > SLOPE_THRESHOLD) {
    return true;
  }
  return false;
}    
    
// ============================================
// コンパスキャリブレーション（簡略版）
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
  
  Serial.println(F("Rotating 15s..."));
  
  unsigned long startTime = millis();
  
  // 時計回り 5秒
  while (millis() - startTime < 5000) {
    compass_state.compass.read();
    updateMinMax(compass_state.compass.m.x, compass_state.compass.m.y, 
                 mx_min, mx_max, my_min, my_max);
    motor_ctrl.setSpeeds(120, -120);
    delay(20);
  }
  
  // 反時計回り 5秒
  startTime = millis();
  while (millis() - startTime < 5000) {
    compass_state.compass.read();
    updateMinMax(compass_state.compass.m.x, compass_state.compass.m.y,
                 mx_min, mx_max, my_min, my_max);
    motor_ctrl.setSpeeds(-120, 120);
    delay(20);
  }
  
  // 時計回り 5秒
  startTime = millis();
  while (millis() - startTime < 5000) {
    compass_state.compass.read();
    updateMinMax(compass_state.compass.m.x, compass_state.compass.m.y,
                 mx_min, mx_max, my_min, my_max);
    motor_ctrl.setSpeeds(120, -120);
    delay(20);
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

}
