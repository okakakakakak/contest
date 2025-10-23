#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <ZumoMotors.h>
#include <Adafruit_TCS34725.h>
#include <LSM303.h>
#include <Pushbutton.h>

// ============================================
// 色定義
// ============================================
#define COLOR_WHITE    0
#define COLOR_BLACK    1
#define COLOR_RED      2
#define COLOR_BLUE     3
#define COLOR_OTHER    4

// ============================================
// 状態定義
// ============================================
#define STATE_INIT              0
#define STATE_SEARCH            1
#define STATE_CHECK_STATIC      2
#define STATE_APPROACH          3
#define STATE_TURN_TO_TARGET    4
#define STATE_WAIT_AFTER_TURN   5
#define STATE_ESCAPE            6
#define STATE_AVOID             7
#define STATE_STOP              8
#define STATE_MOVE              9

// ============================================
// モーター速度定数構造体
// ============================================
struct MotorSpeeds {
  static const int ROTATE     = 140;
  static const int FORWARD    = 140;
  static const int ESCAPE     = 140;
  static const int REVERSE    = -140;
  static const int AVOID_ROT  = 140;
  static const int MOVE       = 140;
  static const int STOP       = 0;
};

// ============================================
// PI制御パラメータ
// ============================================
struct PIController {
  float kp;
  float ti_inv;
  float sum_e;
  
  PIController() : kp(4.0), ti_inv(4.0/1000.0), sum_e(0) {}
  
  void reset() { sum_e = 0; }
};

// ============================================
// 地磁気センサー補正用構造体
// ============================================
struct MagnetometerCalibration {
  float offset_x, offset_y;
  float scale_x, scale_y;
  
  MagnetometerCalibration() : offset_x(0), offset_y(0), scale_x(1.0), scale_y(1.0) {}
};

// ============================================
// コンパス状態構造体
// ============================================
#define HEADING_FILTER_SIZE 5

struct CompassState {
  LSM303 compass;
  MagnetometerCalibration calib;
  float heading_buffer[HEADING_FILTER_SIZE];
  int heading_index;
  float current_heading;
  
  CompassState() : heading_index(0), current_heading(0) {
    for (int i = 0; i < HEADING_FILTER_SIZE; i++) {
      heading_buffer[i] = 0;
    }
  }
  
  void updateHeading(float magnetic_declination);
};

// ============================================
// カラーセンサー状態構造体
// ============================================
struct ColorSensorState {
  Adafruit_TCS34725 tcs;
  unsigned int r_min, g_min, b_min;
  unsigned int r_max, g_max, b_max;
  int current_color;
  int previous_color;
  
  ColorSensorState() : 
    tcs(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X),
    r_min(60), g_min(52), b_min(62),
    r_max(255), g_max(255), b_max(255),
    current_color(COLOR_OTHER), previous_color(COLOR_OTHER) {}
  
  void getRGB(float& r, float& g, float& b);
  void calibrate();
  int identifyColor(int r, int g, int b);
};

// ============================================
// 距離センサー構造体
// ============================================
struct UltrasonicSensor {
  int trig_pin;
  int echo_pin;
  
  UltrasonicSensor(int trig, int echo) : trig_pin(trig), echo_pin(echo) {}
  
  void init();
  int getDistance();
  bool isObjectStatic();
};

// ============================================
// ロボット状態構造体
// ============================================
struct RobotState {
  int mode;
  int previous_mode;
  unsigned long state_start_time;
  unsigned long search_start_time;
  int search_rotation_count;
  bool object_detected_in_search;
  unsigned long time_now;
  unsigned long time_prev;
  
  RobotState() : 
    mode(STATE_INIT), previous_mode(-1),
    state_start_time(0), search_start_time(0),
    search_rotation_count(0), object_detected_in_search(false),
    time_now(0), time_prev(0) {}
  
  void updateTime() {
    time_prev = time_now;
    time_now = millis();
  }
};

// ============================================
// モーター制御構造体
// ============================================
struct MotorController {
  ZumoMotors motors;
  int left_speed;
  int right_speed;
  
  MotorController() : left_speed(0), right_speed(0) {}
  
  void setSpeeds(int left, int right) {
    left_speed = left;
    right_speed = right;
    motors.setSpeeds(left_speed, right_speed);
  }
  
  void stop() {
    setSpeeds(0, 0);
  }
};

// ============================================
// グローバルオブジェクト（必要最小限）
// ============================================
extern Pushbutton button;
extern MotorController motor_ctrl;
extern ColorSensorState color_sensor;
extern CompassState compass_state;
extern UltrasonicSensor ultrasonic;
extern RobotState robot_state;
extern PIController pi_ctrl;

// ============================================
// 定数
// ============================================
extern const float TARGET_HEADING;
extern const float MAGNETIC_DECLINATION;

// ============================================
// 関数プロトタイプ
// ============================================
const char* getModeName(int mode);
void printModeChange();
void printStatus();
void task();
float turnTo(float target_heading);
void calibrationCompassAdvanced();

#endif