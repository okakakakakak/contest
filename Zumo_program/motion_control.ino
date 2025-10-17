#include "definitions.h"

// ============================================
// PI制御による方位制御
// ============================================
float turnTo(float psi_r) {
  float u;
  
  updateHeading();
  
  float e = psi_r - heading_G;
  
  // 角度誤差を-180〜180度の範囲に正規化
  while (e < -180) e += 360;
  while (e > 180) e -= 360;
  
  if (abs(e) > 45.0) {
    // |e|>45° の場合はP制御
    u = KP * e;
    sum_e = 0;
  } else {
    // |e|<=45° の場合はPI制御
    sum_e += TIinv * e * (timeNow_G - timePrev_G);
    sum_e = constrain(sum_e, -50, 50);  // アンチワインドアップ
    u = KP * (e + sum_e);
  }
  
  // 制御入力の制限
  u = constrain(u, -180, 180);
  
  return u;
}