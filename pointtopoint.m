clc;
clear all;
close all;

% Parameters
u_max = 2; % maximum surge velocity
r_max = 0.1; % maximum yaw rate
tspan = [0 100]; % simulation time
Kp = 0.6; % proportional gain for yaw control

% Desired position
x_desired = 50;
y_desired = 50;

% Initial conditions [x, y, psi]
initial_conditions = [0; 0; 0]; % starting at origin, heading = 0

% ODE function for point-to-point control
odefun = @(t, state) point_to_point_control(t, state, x_desired, y_desired, u_max, Kp);

% Solve the ODEs
[t, state] = ode45(odefun, tspan, initial_conditions);

% Plot the trajectory
figure;
plot(state(:,1), state(:,2));
hold on;
plot(x_desired, y_desired, 'rx', 'MarkerSize', 10); % plot desired point
xlabel('X Position');
ylabel('Y Position');
title('Boat Trajectory - Point-to-Point Control');
grid on;

% Animation
for i = 1:length(t)
    plot(state(i,1), state(i,2), 'bo'); % plot current position
    pause(0.1); % pause to create animation effect
end

% Calculate and plot the error over time
errors = sqrt((state(:,1) - x_desired).^2 + (state(:,2) - y_desired).^2);

figure;
plot(t, errors);
xlabel('Time (s)');
ylabel('Position Error');
title('Position Error over Time');
grid on;

