#include "definitions.h"

// ============================================
// PI制御による方位制御
// ============================================
float turnTo(float target_heading) {
  compass_state.updateHeading(MAGNETIC_DECLINATION);
  robot_state.updateTime();
  
  float e = target_heading - compass_state.current_heading;
  
  // 角度誤差を-180〜180度の範囲に正規化
  while (e < -180) e += 360;
  while (e > 180) e -= 360;
  
  float u;
  if (abs(e) > 45.0) {
    // |e|>45° の場合はP制御
    u = pi_ctrl.kp * e;
    pi_ctrl.sum_e = 0;
  } else {
    // |e|<=45° の場合はPI制御
    pi_ctrl.sum_e += pi_ctrl.ti_inv * e * (robot_state.time_now - robot_state.time_prev);
    pi_ctrl.sum_e = constrain(pi_ctrl.sum_e, -50, 50);  // アンチワインドアップ
    u = pi_ctrl.kp * (e + pi_ctrl.sum_e);
  }
  
  // 制御入力の制限
  u = constrain(u, -180, 180);
  
  return u;
}