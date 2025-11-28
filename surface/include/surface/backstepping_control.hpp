#ifndef BACK_AUV_CONTROLLER_HPP
#define BACK_AUV_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <utility>
#include <limits>

//---------------- Global Parameters ----------------
// Vehicle parameters
static const double m         = 148.3571;     // Mass [kg]
static const double I_zz      = 41.980233;    // Yaw inertia [kg·m²]
static const double I_yy      = 41.980233;    // Pitch inertia [kg·m²]

// Added mass terms
static const double X_dot_u   = -4.876161;    // Added mass in surge
static const double Y_dot_v   = -126.324739;  // Added mass in sway
static const double N_dot_r   = -33.46;       // Added mass in yaw
static const double M_dot_q   = -33.46;       // Added mass in pitch

// Hydrodynamic drag (quadratic coefficients)
static const double X_u_absu  = -6.2282;      // Surge drag coefficient
static const double Y_v_absv  = -601.27;      // Sway drag coefficient
static const double N_r_absr  = -632.698957;  // Yaw drag coefficient
static const double M_q_absq  = -632.698957;  // Pitch drag coefficient

// Effective inertias (after adding added mass)
static const double k_N = 0.5;    // Fin effectiveness coefficient
static const double k_Y = 0.2;    // Fin effectiveness coefficient

//---------------------------------------------------

struct OuterLoopOutput {
    double u_d;
    double r_d;
    double u_d_dot;
    double r_d_dot;
};

class OuterBack {
public:
    double K_u, K_yaw;

    OuterBack(double k_u, double k_yaw) : K_u(k_u), K_yaw(k_yaw) {}

    OuterLoopOutput compute(double x_d, double y_d,double yaw_d,
                            double x, double y,
                            double heading,
                            double u, double v, double r,
                            double x_d_dot = 0.0, double y_d_dot = 0.0) 
    {
        // Compute position errors
        double e_x = x_d - x;
        double e_y = y_d - y;
        double dist_sq = e_x * e_x + e_y * e_y;
        double dist = std::sqrt(dist_sq);

        // Avoid division by zero
        if (dist < 1e-6) dist = 1e-6;

        // Desired surge speed and heading
        double u_d = K_u * dist;
        double psi_d = yaw_d;
        double e_heading = psi_d - heading;
        double r_d = K_yaw * e_heading;

        // Error derivatives
        double e_x_dot = x_d_dot - (u * std::cos(heading) - v * std::sin(heading));
        double e_y_dot = y_d_dot - (u * std::sin(heading) + v * std::cos(heading));

        // Derivatives of desired velocities
        double u_d_dot = (K_u / dist) * (e_x * e_x_dot + e_y * e_y_dot);
       // double psi_d_dot = (-e_y * e_x_dot + e_x * e_y_dot) / dist_sq;
       double psi_d_dot =0;
        double r_d_dot = K_yaw * (psi_d_dot - r);

        return {u_d, r_d, u_d_dot, r_d_dot};
    }
};

class InnerBack {
public:
    double K1, K2;

    InnerBack(double k1, double k2) : K1(k1), K2(k2) {}

    std::pair<double, double> compute(double u, double v, double r,
                                      const OuterLoopOutput &outer) 
    {
        // Surge dynamics
        double f1 = ((m - Y_dot_v) * v * r - X_u_absu * std::fabs(u) * u) / (m - X_dot_u);
        double g1 = 1.0 / (m - X_dot_u);

        // Yaw dynamics
        double f2 = (-N_r_absr * std::fabs(r) * r) / (I_zz - N_dot_r);
        double g2 = (k_N * u * u) / (I_zz - N_dot_r);

        // (Sway dynamics exists but no sway actuator, so omitted from control law)

        // Backstepping control inputs
        double thrust = (1.0 / g1) * (-f1 + outer.u_d_dot - K1 * (u - outer.u_d));
        double fin = (1.0 / g2) * (-f2 + outer.r_d_dot - K2 * (r - outer.r_d));

        return std::make_pair(thrust, fin);
    }
};

#endif
