#ifndef PID_AUV_CONTROLLER_HPP
#define PID_AUV_CONTROLLER_HPP
#include <algorithm>
#include <cmath>
#include <limits>

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    // Compute PID control output.
    double compute(double error, double dt) {
        if (dt < std::numeric_limits<double>::epsilon()) return 0.0;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    double compute_yaw(double torque,double current_velocity){
            double deflection= (torque)/((25.2*current_velocity*current_velocity)+0.1);
            double def= std::clamp(deflection,-0.92,0.92);
            return def;
    }

    void reset() {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

#endif