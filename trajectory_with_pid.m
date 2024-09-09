clc;
clear;
close all;

%% Coordinates
x_0 = 0;
y_0 = 0;
psi = 0; % Initial yaw angle
L = 0.1;   % Length of the boat

%% Time parameters
dt = 0.01;
ts = 50;
t = 0:dt:ts;

% Sinusoidal Trajectory Parameters
A = 2;   % Amplitude of the sine wave
k = 0.5; % Frequency of the sine wave
x_speed = 0.1;  % Linear speed in the x-direction

traj_x = [];
traj_y = [];
psi_s = [];
e_x = [];
e_y = [];
e_p = [];

% Initialize PID variables
% Proportional, Integral, Derivative gains for x and y
Kp_x = 0.7; Ki_x = 0; Kd_x = 0.4;
Kp_y = 1; Ki_y = 0.1; Kd_y = 0.05;

% PID gains for yaw
Kp_psi = 0.9; Ki_psi = 0; Kd_psi = 0.5;

% PID error accumulations
int_error_x = 0; int_error_y = 0;
int_error_psi = 0;
prev_error_x = 0; prev_error_y = 0;
prev_error_psi = 0;

for k = 1:length(t)
    %% Desired Sinusoidal Path
    x_des = x_speed * t(k);              % Linear increase in x over time
    y_des = A * sin(k * x_des);           % Sinusoidal y as a function of x
    psi_des = atan2(y_des - y_0, x_des - x_0); % Desired yaw angle to point along the path

    %% Calculate Position and Orientation Error
    error_x = x_des - x_0;
    error_y = y_des - y_0;
    error_psi = psi_des - psi;

    % Store errors for plotting
    e_x(k) = error_x;
    e_y(k) = error_y;
    e_p(k) = error_psi;

    %% PID Controller for X (forward velocity)
    int_error_x = int_error_x + error_x * dt; % Integral term
    der_error_x = (error_x - prev_error_x) / dt; % Derivative term
    u_x = Kp_x * error_x + Ki_x * int_error_x + Kd_x * der_error_x; % PID output for x
    u_x = max(u_x, 0);  % Prevent reverse movement by ensuring u_x is non-negative
    prev_error_x = error_x;

    %% Set sideways velocity to zero (non-holonomic constraint)
    u_y = 0;

    %% PID Controller for Yaw (orientation)
    int_error_psi = int_error_psi + error_psi * dt; % Integral term
    der_error_psi = (error_psi - prev_error_psi) / dt; % Derivative term
    r = Kp_psi * error_psi + Ki_psi * int_error_psi + Kd_psi * der_error_psi; % PID output for yaw
    prev_error_psi = error_psi;

    %% Kinematics
    % Jacobian matrix (rotation matrix)
    J = [cos(psi) -sin(psi) 0;
         sin(psi)  cos(psi) 0;
         0         0        1];

    vel = [u_x; u_y; r];
    out = J * vel;

    x_dot = out(1);
    y_dot = out(2);
    psi_dot = out(3);

    %% Update Position and Orientation
    x_0 = x_0 + x_dot * dt;
    y_0 = y_0 + y_dot * dt;
    psi = psi + psi_dot * dt;

    % Store trajectory
    traj_x(k) = x_0;
    traj_y(k) = y_0;
    psi_s(k) = psi;
end
%% Animation of the Boat
figure;
for i = 1:length(traj_x)
    % Extract current positions and orientations
    x_leader = traj_x(i);
    y_leader = traj_y(i);
    psi_leader = psi_s(i);

    % Define the boat as a triangle
    boat_length = L;
    boat_width = 0.5 * L; % Width proportional to length

    % Define vertices of the boat shape
    boat_shape = [boat_length, 0; -boat_length/2, boat_width/2; -boat_length/2, -boat_width/2];

    % Rotation matrix for the boat's orientation
    R = [cos(psi_leader) -sin(psi_leader);
         sin(psi_leader) cos(psi_leader)];

    % Rotate and translate the boat shape to the current position
    rotated_boat = (R * boat_shape')';

    % Shift the boat to its current position
    x_boat = rotated_boat(:, 1) + x_leader;
    y_boat = rotated_boat(:, 2) + y_leader;

    % Plot the trajectory
    plot(traj_x(1:i), traj_y(1:i), 'r-', 'LineWidth', 1.5);
    hold on;

    % Plot the boat as a triangle
    fill(x_boat, y_boat, 'b'); % Blue triangle for the boat

    % Set axis limits and labels
    axis equal;
    title('Boat Animation with Orientation Control');
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on;

    pause(0.1);
    hold off;
end
%% Plotting the Trajectory
figure;
plot(traj_x, traj_y, '-o');
hold on;
plot(x_speed * t, A * sin(k * x_speed * t), 'r--', 'LineWidth', 2);  % Plot the desired sinusoidal path
xlabel('X Position');
ylabel('Y Position');
title('Boat Trajectory Following a Sinusoidal Path with PID Control');
legend('Actual Trajectory', 'Desired Sinusoidal Path');
grid on;

%% Error Plot
figure;
plot(t, e_x, '-', 'LineWidth', 4);
hold on;
plot(t, e_y, '-.', 'LineWidth', 4);
plot(t, e_p, ':', 'LineWidth', 4);

legend('error in x, [m]', 'error in y, [m]', 'error in yaw, [rad]');
xlabel('t, [s]');
ylabel('errors, [units]');
grid on;
