#ifndef CASCADED_AUV_CONTROLLER_HPP
#define CASCADED_AUV_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <limits>

namespace auv {

//---------------- Global Parameters ----------------
// Vehicle parameters
static const double m         = 148.3571;     // Mass [kg]
static const double I_zz      = 41.980233;    // Yaw inertia [kg·m²]
static const double I_yy      = 41.980233;    // Pitch inertia [kg·m²]

// Added mass terms
static const double X_dot_u   = -4.876161;    // Added mass in surge
static const double N_dot_r   = -33.46;       // Added mass in yaw
static const double M_dot_q   = -33.46;       // Added mass in pitch

// Hydrodynamic drag (quadratic coefficients)
static const double X_u_absu  = -6.2282;      // Surge drag coefficient
static const double N_r_absr  = -632.698957;  // Yaw drag coefficient
static const double M_q_absq  = -632.698957;  // Pitch drag coefficient

// Effective inertias (after adding added mass)
static const double m_u       = m - X_dot_u;         // Effective surge inertia (≈153.2333)
static const double I_zz_star = I_zz - N_dot_r;        // Effective yaw inertia (≈75.440233)
static const double I_yy_star = I_yy - M_dot_q;        // Effective pitch inertia (≈75.440233)

//---------------- PID Controller Class ----------------
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

    void reset() {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

//---------------- Outer Loop Controllers ----------------
// OuterSurgeController: Uses a PID on surge (position) error to generate a desired surge velocity.
class OuterSurgeController {
public:
    OuterSurgeController(double kp, double ki, double kd)
        : pid_(kp, ki, kd) {}

    double computeDesiredVelocity(double desired_pos, double current_pos, double dt) {
        double pos_error = desired_pos - current_pos;
        return pid_.compute(pos_error, dt);
    }

private:
    PIDController pid_;
};

// OuterAngleController: Uses a PID on angle error to compute a desired angular rate.
// This class is used for both yaw and pitch.
class OuterAngleController {
public:
    OuterAngleController(double kp, double ki, double kd)
        : pid_(kp, ki, kd) {}

    double computeDesiredAngularRate(double desired_angle, double current_angle, double dt) {
        double angle_error = desired_angle - current_angle;
        // Normalize error to [-pi, pi]
        while (angle_error > M_PI)  { angle_error -= 2.0 * M_PI; }
        while (angle_error < -M_PI) { angle_error += 2.0 * M_PI; }
        return pid_.compute(angle_error, dt);
    }

private:
    PIDController pid_;
};

//---------------- Inner Loop Controllers with Feedback Linearization ----------------
// AUVInnerController: Inner loop for surge and yaw channels.
class AUVInnerController {
public:
    AUVInnerController(double kp_u, double ki_u, double kd_u,
                       double kp_yaw, double ki_yaw, double kd_yaw)
        : pid_u_(kp_u, ki_u, kd_u),
          pid_yaw_(kp_yaw, ki_yaw, kd_yaw)
    {}

    // Compute surge thrust and yaw torque.
    // Inputs:
    //   u_d: desired surge velocity from outer loop.
    //   r_d: desired yaw rate from outer loop.
    //   u: current surge velocity.
    //   r: current yaw rate.
    //   dt: time step.
    // Outputs:
    //   T: computed thrust command.
    //   tau_yaw: computed yaw torque command.
    void computeControl(double u_d, double u, double r_d, double r, double dt,
                        double &T, double &tau_yaw) {
        // Virtual control inputs (PID on velocity errors)
        double v_u = pid_u_.compute(u_d - u, dt);
        double v_r = pid_yaw_.compute(r_d - r, dt);

        // Feedback linearization terms:
        // Surge: f_u = -m_u*u*r + X_u_absu*|u|*u.
        double f_u = -m_u * u * r + X_u_absu * std::abs(u) * u;
        // Yaw: f_r = m_u*u*r + N_r_absr*|r|*r.
        double f_r = m_u * u * r + N_r_absr * std::abs(r) * r;

        T = m_u * v_u - f_u;
        tau_yaw = I_zz_star * v_r + f_r;
    }

private:
    PIDController pid_u_;
    PIDController pid_yaw_;
};

// AUVPitchInnerController: Inner loop for pitch channel.
class AUVPitchInnerController {
public:
    AUVPitchInnerController(double kp_pitch, double ki_pitch, double kd_pitch)
        : pid_pitch_(kp_pitch, ki_pitch, kd_pitch)
    {}

    // Compute pitch torque command.
    // Inputs:
    //   q_d: desired pitch rate (from outer loop).
    //   q: current pitch rate.
    //   dt: time step.
    // Outputs:
    //   tau_pitch: computed pitch torque command.
    void computeControl(double q_d, double q, double dt,
                        double &tau_pitch) {
        // Virtual control input: PID on pitch rate error.
        double v_pitch = pid_pitch_.compute(q_d - q, dt);

        // Feedback linearization for pitch:
        // f_q = M_q_absq * |q| * q.
        double f_q = M_q_absq * std::abs(q) * q;

        // Control law: I_yy_star * dot(q) = tau_pitch - f_q  =>  tau_pitch = I_yy_star * v_pitch - f_q.
        tau_pitch = I_yy_star * v_pitch - f_q;
    }

private:
    PIDController pid_pitch_;
};

//---------------- Cascaded AUV Controller ----------------
// Combines outer and inner loops for surge, yaw, and pitch channels.
class CascadedAUVController {
public:
    // Constructor: supply gains for outer and inner loops.
    CascadedAUVController(
         // Outer loop gains
         double kp_pos, double ki_pos, double kd_pos,
         double kp_yawAngle, double ki_yawAngle, double kd_yawAngle,
         double kp_pitchAngle, double ki_pitchAngle, double kd_pitchAngle,
         // Inner loop gains for surge and yaw
         double kp_u, double ki_u, double kd_u,
         double kp_yaw, double ki_yaw, double kd_yaw,
         // Inner loop gains for pitch
         double kp_pitch, double ki_pitch, double kd_pitch)
         : outerSurge_(kp_pos, ki_pos, kd_pos),
           outerYaw_(kp_yawAngle, ki_yawAngle, kd_yawAngle),
           outerPitch_(kp_pitchAngle, ki_pitchAngle, kd_pitchAngle),
           innerController_(kp_u, ki_u, kd_u, kp_yaw, ki_yaw, kd_yaw),
           innerPitchController_(kp_pitch, ki_pitch, kd_pitch)
    {}

    // Compute overall control.
    // Inputs:
    //   desired_pos: desired surge position.
    //   desired_yaw: desired yaw angle [rad].
    //   desired_pitch: desired pitch angle [rad].
    //   current_pos: current surge position.
    //   current_yaw: current yaw angle [rad].
    //   current_pitch: current pitch angle [rad].
    //   u: current surge velocity.
    //   r: current yaw rate.
    //   q: current pitch rate.
    //   dt: time step.
    // Outputs:
    //   T: computed thrust command.
    //   tau_yaw: computed yaw torque command.
    //   tau_pitch: computed pitch torque command.
    void computeControl(double desired_pos, double desired_yaw, //double desired_pitch,
                        double current_pos, double current_yaw, //double current_pitch,
                        double u, double r,double dt,// double q, 
                        double &T, double &tau_yaw ) {//double &tau_pitch
        double u_d = outerSurge_.computeDesiredVelocity(desired_pos, current_pos, dt);
        double r_d = outerYaw_.computeDesiredAngularRate(desired_yaw, current_yaw, dt);
       // double q_d = outerPitch_.computeDesiredAngularRate(desired_pitch, current_pitch, dt);
        innerController_.computeControl(u_d, u, r_d, r, dt, T, tau_yaw);
       // innerPitchController_.computeControl(q_d, q, dt, tau_pitch);
    }

private:
    OuterSurgeController outerSurge_;
    OuterAngleController outerYaw_;
    OuterAngleController outerPitch_;
    AUVInnerController innerController_;
    AUVPitchInnerController innerPitchController_;
};

//---------------- Fin Mapping Function ----------------
// Converts the desired yaw torque (tau_yaw) to a fin deflection command.
// Uses the relationship:
//   tau_fin = 0.5 * rho * V^2 * S * CLA * l * delta  ==>  delta = (2*tau_fin) / (rho * V^2 * S * CLA * l)
inline double computeFinDeflection(double tau_desired, double V) {
    const double rho = 1000.0;  // Fluid density [kg/m^3]
    const double S = 0.0244;    // Fin area [m^2]
    const double CLA = 4.13;    // Lift coefficient slope
    const double l = 0.5;       // Lever arm [m]
    double V_eff = std::max(V, 0.1);
    double delta = (2.0 * tau_desired) / (0.5 * rho * V_eff * V_eff * S * CLA * l);
    const double maxDelta = 0.92;  // Maximum fin deflection [rad]
    return std::clamp(delta, -maxDelta, maxDelta);
}

}  // namespace auv

#endif  // CASCADED_AUV_CONTROLLER_HPP
