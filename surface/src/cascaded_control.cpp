// cascaded_control.cpp

#include "/home/arash/ros2_ws/src/surface/include/surface/cascaded_control.hpp"

// ---------------- PID Controller ------------------

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

void PIDController::reset() {
    prev_error_ = 0.0;
    integral_ = 0.0;
}

double PIDController::compute(double error, double dt) {
    if (dt <= 1e-6) return 0.0;

    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

// ---------------- Cascaded Controller ------------------

CascadedController::CascadedController(double kp_pos, double ki_pos, double kd_pos,
                                       double kp_ori, double ki_ori, double kd_ori)
    : pid_position_(kp_pos, ki_pos, kd_pos),
      pid_orientation_(kp_ori, ki_ori, kd_ori),
      desired_velocity_(0.0),
      control_input_(0.0) {}

void CascadedController::reset() {
    pid_position_.reset();
    pid_orientation_.reset();
    desired_velocity_ = 0.0;
    control_input_ = 0.0;
}

void CascadedController::compute_position_control(double desired_pos, double current_pos, double dt) {
    double position_error = desired_pos - current_pos;
    desired_velocity_ = pid_position_.compute(position_error, dt);
}

double CascadedController::compute_orientation_control(double desired_angle, double current_angle, double dt) {
    double angle_error = desired_angle - current_angle;

    // Normalize to [-pi, pi]
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    control_input_ = pid_orientation_.compute(angle_error, dt);
    return control_input_;
}

double CascadedController::get_desired_velocity() const {
    return desired_velocity_;
}

double CascadedController::get_control_input() const {
    return control_input_;
}
