% Function for Point-to-Point Control

function dstate = point_to_point_control(t, state, x_desired, y_desired, u_max, Kp)
    % Current position and heading
    x = state(1);
    y = state(2);
    psi = state(3);
    r_max = 0.1;

    % Calculate the error in position
    error_x = x_desired - x;
    error_y = y_desired - y;

    % Desired heading
    psi_desired = atan2(error_y, error_x);

    % Heading error
    heading_error = psi_desired - psi;

    % Control input (yaw rate)
    r = Kp * heading_error;
    
    % Ensure r does not exceed max yaw rate
    r = max(min(r, r_max), -r_max);

    % Constant surge velocity
    u = u_max;

    % State derivatives
    dstate = [u * cos(psi); u * sin(psi); r];
    
end