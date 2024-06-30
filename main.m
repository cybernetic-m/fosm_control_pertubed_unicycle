%% MAIN

% Title: FOSM-AL Controls comparison of a perturbed Unicycle model
% Author: Massimo Romano

% In the main you can test the Approximate Linearization control 
% or the FOSM control in the case of a Pertubed Unicycle Model

clear all
close all
clc

% Add the functions directory to the MATLAB path
addpath('utils');
addpath('controller');

% Simulation Parameters
T_sim = 10; % Total time of simulation
dt = 0.001;  % Update interval of time

%% Trajectory 
% In this part you can choose the trajectory for the experiment
% 1) Circle
% 2) Lemniscate (Infinity) Curve
 
choice = 'Lemniscate'; % CHANGE THIS!!!
[x_d, y_d, x_dot, y_dot, x_ddot, y_ddot, t] = trajectory (choice);

disp('Trajectory initialization...');

%% INITIAL CONDITIONS ON ROBOT AND DISTURBANCES
% Initial Conditions
x_0 = 1.3;
y_0 = 0;
theta_0 = 0;

% Decomment the choice of perturbation that you want

% Zero Perturbation
d1 = zeros(1, T_sim/dt);
d2 = zeros(1, T_sim/dt);

% Small Perturbation
%d1 = 0.06 * cos(t) + 0.2;
%d2 = 0.05 * sin(t) + 0.2;

% Strong Perturbation
%d1 = 0.06 * cos(t) + 1;
%d2 = 0.05 * sin(t) + 1;


%% Control with Approximate Linearization method

[v_al, omega_al, x_robot_al, y_robot_al, theta_robot_al, e_al] = al_control(x_0, y_0, theta_0, d1, d2, T_sim, dt, x_d, y_d, x_dot, y_dot, x_ddot, y_ddot);
    
disp('Approximate Linearization Control...');


%% Control with First-Order Sliding Mode (FOSM)

[v_fosm, omega_fosm, x_robot_fosm, y_robot_fosm, theta_robot_fosm, e_fosm] = fosm_control(x_0, y_0, theta_0, d1, d2, T_sim, dt, x_d, y_d, x_dot, y_dot, x_ddot, y_ddot);
    
disp('FOSM Control...');


%% PLOT SECTION

t = 1:length(x_robot_al);

figure;

subplot(2, 1, 1);
plot(t, e_al(1, :));
xlabel('Time (t)');
ylabel('e_x');
title('Error in x with AL');
grid on;
axis tight; % Adjust the axis scaling

subplot(2, 1, 2);
plot(t, e_fosm(1, :));
xlabel('Time (t)');
ylabel('e_x');
title('Error in x with FOSM');
grid on;
axis tight; % Adjust the axis scaling


figure;

subplot(2, 1, 1);
plot(t, e_al(2, :));
xlabel('Time (t)');
ylabel('e_y');
title('Error in y with AL');
grid on;
axis tight; % Adjust the axis scaling

subplot(2, 1, 2);
plot(t, e_fosm(2, :));
xlabel('Time (t)');
ylabel('e_y');
title('Error in y with FOSM');
grid on;
axis tight; % Adjust the axis scaling

figure;

subplot(2, 1, 1);
plot(t, e_al(3, :));
xlabel('Time (t)');
ylabel('e_\theta');
title('Error in \theta with AL');
grid on;
axis tight; % Adjust the axis scaling

subplot(2, 1, 2);
plot(t, e_fosm(3, :));
xlabel('Time (t)');
ylabel('e_\theta');
title('Error in  \theta with FOSM');
grid on;
axis tight; % Adjust the axis scaling

%% PLOT Trajectories

figure;

subplot(1, 2, 1); % Divides the figure into 1 row, 2 columns, and selects the first subplot
plot(x_d, y_d, 'r-', 'LineWidth', 2); 
hold on;
plot(x_robot_al, y_robot_al, 'g-.', 'LineWidth', 2);
xlabel('x');
ylabel('y');
title('Trajectories');
legend('Desired Trajectory', 'AL control');
axis equal;
grid on;

% Second subplot for the robot trajectories
subplot(1, 2, 2); % Divides the figure into 1 row, 2 columns, and selects the second subplot
plot(x_d, y_d, 'r-', 'LineWidth', 2); 
hold on;
plot(x_robot_fosm, y_robot_fosm, 'g-.', 'LineWidth', 2);
xlabel('x');
ylabel('y');
title('Trajectories');
legend('Desired Trajectory', 'FOSM control');
axis equal;
grid on;

% Animation Parameters
figure;
axis equal;
hold on;
plot(x_d, y_d, 'r--'); % Plot desired trajectory for FOSM Control
h_robot_fosm = plot(x_robot_fosm(1), y_robot_fosm(1), 'bo'); % Plot robot's position for FOSM Control
h_robot_al = plot(x_robot_al(1), y_robot_al(1), 'ro'); % Plot robot's position for AL Control

% Define a smaller rectangle for FOSM Control
rect_length_fosm = 0.1; % Length of the rectangle
rect_width_fosm = 0.05;  % Width of the rectangle

% The vertices of the rectangle for FOSM Control
rect_x_fosm = [-rect_length_fosm/2, rect_length_fosm/2, rect_length_fosm/2, -rect_length_fosm/2];
rect_y_fosm = [-rect_width_fosm/2, -rect_width_fosm/2, rect_width_fosm/2, rect_width_fosm/2];

% Define a smaller rectangle for AL Control
rect_length_al = 0.1; % Length of the rectangle
rect_width_al = 0.05;  % Width of the rectangle

% The vertices of the rectangle for AL Control
rect_x_al = [-rect_length_al/2, rect_length_al/2, rect_length_al/2, -rect_length_al/2];
rect_y_al = [-rect_width_al/2, -rect_width_al/2, rect_width_al/2, rect_width_al/2];

% Initialize the rectangles for FOSM Control and AL Control
h_rectangle_fosm = fill(rect_x_fosm, rect_y_fosm, 'y'); % Robot as a rectangle for FOSM Control
h_rectangle_al = fill(rect_x_al, rect_y_al, 'g'); % Robot as a rectangle for AL Control

% Set plot title and axis labels
title('Unicycle Robot Animation with AL and FOSM Control');
xlabel('X Position');
ylabel('Y Position');

% Add legend
legend('Desired Trajectory', 'None', 'None', 'Robot Position (FOSM)', 'Robot (AL)');

% Set axis limits to dezoom (zoom out)
xlim([-1.5, 1.5]); % Adjust limits as needed
ylim([-1.2, 2]); % Adjust limits as needed

% Animation Loop
for k = 1:min(length(x_robot_fosm), length(x_robot_al))
    % Update robot's position for FOSM Control
    set(h_robot_fosm, 'XData', x_robot_fosm(k), 'YData', y_robot_fosm(k));
    % Update rectangle vertices for FOSM Control
    theta_fosm = theta_robot_fosm(k);
    R_fosm = [cos(theta_fosm), -sin(theta_fosm); sin(theta_fosm), cos(theta_fosm)];
    rotated_rectangle_fosm = R_fosm * [rect_x_fosm; rect_y_fosm];
    set(h_rectangle_fosm, 'XData', x_robot_fosm(k) + rotated_rectangle_fosm(1,:), ...
                         'YData', y_robot_fosm(k) + rotated_rectangle_fosm(2,:));

    % Update robot's position for AL Control
    set(h_robot_al, 'XData', x_robot_al(k), 'YData', y_robot_al(k));
    % Update rectangle vertices for AL Control
    theta_al = theta_robot_al(k);
    R_al = [cos(theta_al), -sin(theta_al); sin(theta_al), cos(theta_al)];
    rotated_rectangle_al = R_al * [rect_x_al; rect_y_al];
    set(h_rectangle_al, 'XData', x_robot_al(k) + rotated_rectangle_al(1,:), ...
                       'YData', y_robot_al(k) + rotated_rectangle_al(2,:));

    drawnow;
    pause(0.01); % Adjust as needed for desired animation speed
end

hold off;