#include "/home/arash/ros2_ws/src/surface/include/surface/controller.hpp"

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

double PIDController::compute(double error, double dt) {
    if (dt < 1e-6) return 0.0;  // Prevent division by zero

    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}