#include "pid_controller.h"
#include <assert.h>
#include <iostream>
#include <math.h>

namespace shenlan {
namespace control {

PIDController::PIDController(const double kp, const double ki,
                             const double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
}

// /**to-do**/ 计算 PID 输出，实现PID控制
double PIDController::Control(const double error, const double dt) {
    if (dt <= 0) {
        return previous_output_;
    }

    double current_output = 0.0;
    double differential;

    if (std::fabs(integral_) > 5) {
        PIDController::Reset();
    }
    
    if (first_hit_) {
        first_hit_ = false;
    } else {
        differential = (error - previous_error_) / dt;
    }

    integral_ = integral_ + error * dt;
    current_output = kp_ * error + ki_ * integral_ + kd_ * differential;
    previous_error_ = error;
    previous_output_ = current_output;
    return current_output;
}

// /**to-do**/ 重置PID参数
void PIDController::Reset() {

    integral_ = 0.0;
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    first_hit_ = true;  
}

}  // namespace control
}  // namespace shenlan
