clc;
clear;
close all;

%% Coordinates

% Initial Position
x_0 = 0;
y_0 = 0;
psi = 0; % Yaw angle
L = 0.01; % Length of the boat

% Desired Position
x_des = 12;
y_des = 8;
psi_des = pi/6;

%% Time parameters
dt = 0.01;
ts = 25;
t = 0:dt:ts;

traj_x = [];
traj_y = [];
psi_s = [];
e_x = [];
e_y=[];
e_p=[];

%% Kinematics
% u: linear velocity, r: yaw turn rate

for k = 1:length(t)
    %% Calculating Error
    error = [(x_des - x_0); (y_des - y_0)];
    error_psi = psi_des - psi;

    % Stop controller if error is small
    % if norm(error) < 0.01 && abs(error_psi) < 0.01
    %     break;
    % end

    %% Controller Design
    Kp = 0.6;
    Kp_psi = 1;
    u = Kp * error(1);
    r = Kp_psi * error_psi;
    u = max(u, 0);  % Prevent reverse movement by ensuring u_x is non-negative
    % Non-holonomic constraint: lateral velocity (v) is 0
    v = 0; 

    % Jacobian matrix (rotation matrix)
    J = [cos(psi) -sin(psi) 0;
         sin(psi)  cos(psi) 0;
         0         0        1];

    vel = [u; v; r];

    out = J * vel;
    x_dot = out(1);
    y_dot = out(2);
    psi_dot = out(3);

    %% Current Position
    x_0 = x_0 + x_dot * dt;
    y_0 = y_0 + y_dot * dt;
    psi = psi + psi_dot * dt;

    % Wrap psi angle to [-pi, pi]
    psi = mod(psi + pi, 2*pi) - pi;

    % Store trajectory
    traj_x(k) = x_0;
    traj_y(k) = y_0;
    psi_s(k) = psi;
    e_x(k)=error(1);
    e_y(k)=error(2);
    e_p(k)=error_psi;
    %ero(k)=[e_x(k),e_y(k),e_p(k)];

end

%% Plotting
% Plot the trajectory
figure;
plot(traj_x, traj_y, '-o');
hold on;
plot(x_des, y_des, 'rx', 'MarkerSize', 10, 'LineWidth', 2);  % Desired point
xlabel('X Position');
ylabel('Y Position');
title('Robot Trajectory');
%legend('Trajectory', 'Desired Position');
grid on;

%% Animation
figure;
for i = 1:length(traj_x)
    % Extract current positions and orientations
    x_leader = traj_x(i);
    y_leader = traj_y(i);
    psi_leader = psi_s(i);

    % Plot the trajectories
    plot(traj_x(1:i), traj_y(1:i), 'r-', 'LineWidth', 1.5);
    hold on;

    % Plot the leader robot (boat)
    plot(x_leader, y_leader, 'diamond', 'MarkerSize', 10); % Plot the center of the robot
   % plot(x_leader + L/2*cos(psi_leader + pi/2), y_leader + L/2*sin(psi_leader + pi/2), 'b.', 'MarkerSize', 2); % Plot the left wheel
   % plot(x_leader - L/2*cos(psi_leader + pi/2), y_leader - L/2*sin(psi_leader + pi/2), 'b.', 'MarkerSize', 2); % Plot the right wheel

    % axis equal;
    title('Boat Animation');
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on;
    pause(0.1);
end

% 
% 
% 
% 
%% Error Plot
% Assuming 'e' is previously calculated error between leader and follower
figure
plot(t, e_x,'-',LineWidth=4)
hold on
plot(t, e_y,'-.',LineWidth=4)
plot(t, e_p,':',LineWidth=4)

legend('error in x, [m]', 'error in y, [m]', 'error in yaw, [rad]');
%set(gca, 'fontsize', 24)
xlabel('t, [s]');
ylabel('errors, [units]');
% % % 
% % %% Distance Calculation Function (helper function)
% % function L_ij = distance(eta, eta_2)
% %     % Calculate x and y position difference
% %     dx = eta(1) - eta_2(1);
% %     dy = eta(2) - eta_2(2);
% %     % Calculate distance using norm
% %     L_ij = sqrt(dx^2 + dy^2);
% % end
