clc;
clear;
close all;

%% Initial Position
x_0 = 0;
y_0 = 0;
psi = 0; % Initial yaw angle
L = 0.2;   % Length of the boat

%% Desired Position and Orientation
x_des = 6;
y_des = 5;
psi_des = pi/3;

%% Obstacle Location
o_x=4;
o_y=1;
o_x_1=5;
o_y_1=4;


%% Time parameters
dt = 0.01;
ts = 35;
t = 0:dt:ts;
%% Initializing array
traj_x = [];
traj_y = [];
psi_s = [];
e_x = [];
e_y=[];
e_p=[];

%% Initialize PID variables

Kp_x = 2; Ki_x = 0.28; Kd_x = 0.267;  % Proportional, Integral, Derivative gains for x and y

Kp_y = 0.91; Ki_y = 0.1; Kd_y = 0.3;

Kp_psi = 1; Ki_psi = 0.2; Kd_psi = 0.316;

% PID error accumulations
int_error_x = 0; int_error_y = 0;int_error_psi = 0;

prev_error_x = 0; prev_error_y = 0;prev_error_psi = 0;

error_y_max=y_des - y_0; % Maximum Possible Position error for Y

for k = 1:length(t)
    %% Calculate Position and Orientation Error
    error_x = x_des - x_0;
    error_y = y_des - y_0;
    error_psi = psi_des - psi;
    error_y_norm=(error_y/error_y_max)*(pi/2);

    % Stop controller if error is small
    

    %% Storing Error and Trajectory

    e_x(k)=error_x;
    e_y(k)=error_y;
    e_p(k)=error_psi;
   

    traj_x(k) = x_0;
    traj_y(k) = y_0;
    psi_s(k) = psi;

    %% Lateral Position Error and Constraints

    u_y = 0;
   % der_error_y=(error_y - prev_error_y)/dt;
    %int_error_y = int_error_y + error_y * dt;
    %prev_error_y=error_y;

    %% PID Controller for X (forward velocity)
    int_error_x = int_error_x + error_x * dt; % Integral term
   
    der_error_x = (error_x - prev_error_x) / dt; % Derivative term

    u_x = Kp_x * error_x + Ki_x * int_error_x + Kd_x * der_error_x; 
 
    u_x = u_x + Kp_y * error_y; % Incorporating Y_error Also
    
    u_x = max(u_x, 0);  % Prevent reverse movement by ensuring u_x is non-negative
    
    prev_error_x = error_x; % Updating Error
   

    %% PID Controller for Yaw (orientation)
    int_error_psi = int_error_psi + error_psi * dt; % Integral term

    der_error_psi = (error_psi - prev_error_psi) / dt; % Derivative term

    max_yaw_rate = 1.5;

    r = Kp_psi * error_psi + Ki_psi * int_error_psi + Kd_psi * der_error_psi; % PID output for yaw
    
    r = r + Kp_y*error_y_norm;  % Incorporating Y_error Also
    if r>max_yaw_rate
     r=max_yaw_rate;

    end

     prev_error_psi = error_psi;

    %% Kinematics
    % Jacobian matrix (rotation matrix)
    J = [cos(psi) -sin(psi) 0;
         sin(psi)  cos(psi) 0;
         0         0        1];

    vel = [u_x; u_y; r];
    out = J * vel;

    x_dot = out(1);
   % y_dot = out(2);
    y_dot=u_x*sin(psi);
    psi_dot = out(3);

  

    %% AFP
    K_rep=440;
    d_safe=1.2;
    distance_min=0.01;
    distance=sqrt((x_0-o_x)^2+(y_0-o_y)^2);
    
    if distance <= d_safe
        U_rep = 0.5 * K_rep * ((1 / distance) - (1 / d_safe))^2;
        F_rep = -K_rep * ((1 / distance) - (1 / d_safe));
        F_rep_max = -K_rep * ((1 / distance_min) - (1 / d_safe));
        psi_dot =   (F_rep / F_rep_max) * (pi / 6);
    end



   % distance_1=sqrt((x_0-o_x_1)^2+(y_0-o_y_1)^2);
   % 
   %  if distance_1 <=d_safe
   % 
   %  U_rep_1=0.5*K_rep*((1/distance_1)-(1/d_safe))^2;
   % 
   %  F_rep_1=-K_rep*((1/distance_1)-(1/d_safe));
   % 
   %  F_rep_1_max=-K_rep*((1/distance_min)-(1/d_safe));
   % 
   %  psi_dot = out(3)+ (F_rep_1/F_rep_1_max)*(pi/6);
   % 
   %  else
   %       F_rep_1=0;
   %        psi_dot=out(3);
   %  end
    %% Update Position and Orientation
    x_0 = x_0 + x_dot * dt;
    y_0 = y_0 + y_dot * dt;
    psi = psi + psi_dot * dt;

   if norm([error_x, error_y]) < 0.25 && abs(error_psi) < 0.25
        break;
   end
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
    %% Plotting Obstacle
    r=0.25;
    theta=linspace(0,2*pi,100);
    fill(o_x+r*cos(theta),o_y+r*sin(theta),'-');
    hold on;
    % r_1=0.30;
    % theta=linspace(0,2*pi,100);
    % fill(o_x_1+r_1*cos(theta),o_y_1+r_1*sin(theta),'-');
    % hold on;

    %% Plot the trajectory
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
% %% Plotting the Trajectory
figure;
plot(traj_x, traj_y, '-o');
hold on;
plot(x_des, y_des, 'rx', 'MarkerSize', 10, 'LineWidth', 2);  % Desired point
xlabel('X Position');
ylabel('Y Position');
title('Boat Trajectory with PID Control for Position and Yaw');
legend('Trajectory', 'Desired Position');
grid on;

%% Error Plot
% Assuming 'e' is previously calculated error between leader and follower
% figure
% plot(t, e_x,'-',LineWidth=4)
% hold on
% plot(t, e_y,'-.',LineWidth=4)
% plot(t, e_p,':',LineWidth=4)
% 
% legend('error in x, [m]', 'error in y, [m]', 'error in yaw, [rad]');
% %set(gca, 'fontsize', 24)
% xlabel('t, [s]');
% ylabel('errors, [units]');