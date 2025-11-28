#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

class PIDController {
public:
    PIDController(double kp, double ki, double kd);  // Constructor
    

    double compute(double error, double dt);  // Compute method

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

#endif  // CONTROLLER_HPP
