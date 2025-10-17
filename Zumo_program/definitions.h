#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <ZumoMotors.h>
#include <Adafruit_TCS34725.h>
#include <LSM303.h>
#include <Pushbutton.h>

// ============================================
// 色定義
// ============================================
#define WHITE    0
#define BLACK    1
#define RED      2
#define BLUE     3
#define OTHER    4

// ============================================
// 状態定義
// ============================================
#define INIT              0
#define SEARCH            1
#define CHECK_STATIC      2
#define APPROACH          3
#define TURN_TO_TARGET    4
#define WAIT_AFTER_TURN   5
#define ESCAPE            6
#define AVOID             7
#define STOP              8
#define MOVE              9
#define GOAL              10

// ============================================
// モーター速度定数
// ============================================
extern const int SPEED_ROTATE;
extern const int SPEED_FORWARD;
extern const int SPEED_ESCAPE;
extern const int SPEED_REVERSE;
extern const int SPEED_AVOID_ROT;
extern const int SPEED_MOVE;
extern const int SPEED_STOP;

// ============================================
// 目標方位角設定
// ============================================
extern const float TARGET_HEADING;

// ============================================
// PI制御定数
// ============================================
extern const float KP;
extern const float TIinv;

// ============================================
// グローバル変数宣言
// ============================================
extern Pushbutton button;
extern ZumoMotors motors;
extern int motorL, motorR;

extern Adafruit_TCS34725 tcs;
extern float r, g, b;
extern unsigned int r_min, g_min, b_min;
extern unsigned int r_max, g_max, b_max;
extern int color, prevColor;

extern LSM303 compass;
extern float mx, my;
extern float heading_G;

extern const int trig;
extern const int echo;
extern int dist;

extern int mode;
extern int prevMode;
extern unsigned long start_time;
extern unsigned long searchStartTime;
extern unsigned long timeNow_G, timePrev_G;
extern int searchRotationCount;
extern bool objectDetectedInSearch;

extern float sum_e;

// ============================================
// 関数プロトタイプ (sensors.cpp)
// ============================================
int identify_color(int r, int g, int b);
int distance();
bool isCupStatic();
void calibrationCompass();
void calibrationColorSensor();
void getRGB(float& r0, float& g0, float& b0);
void updateHeading();

// ============================================
// 関数プロトタイプ (motion_control.cpp)
// ============================================
float turnTo(float psi_r);

// ============================================
// 関数プロトタイプ (state_machine.cpp)
// ============================================
const char* getModeName(int mode);
void printModeChange();
void printStatus();
void task();


#endif
