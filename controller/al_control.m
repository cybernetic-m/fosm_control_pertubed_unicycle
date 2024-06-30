%% APPROXIMATE LINEARIZATION CONTROL

% Title: FOSM-AL Controls comparison of a perturbed Unicycle model
% Author: Massimo Romano

% This function implement the "Approximate linearization" Control

function [v, omega, x_robot, y_robot, theta_robot, e] = al_control(x_0, y_0, theta_0, d_1, d_2, T_sim, dt, x_d, y_d, x_dot, y_dot, x_ddot, y_ddot)

    % Tmp Variables
    d1 = d_1;
    d2 = d_2;

    % Control Parameters 
    k1 = 10;
    k2 = 100;
    k3 = 10;
    
    % Initialization
    x_robot = zeros(1, T_sim/dt);
    y_robot = zeros(1, T_sim/dt);
    theta_robot = zeros(1, T_sim/dt);
    e1 = zeros(1, T_sim/dt);
    e2 = zeros(1, T_sim/dt);
    e3 = zeros(1, T_sim/dt);
    omega = zeros(1, T_sim/dt);
    v = zeros(1, T_sim/dt);
    v_d = zeros(1, T_sim/dt);
    omega_d = zeros(1, T_sim/dt);
    e = zeros(3, T_sim/dt);
    
    % Desired Orientation
    theta_d = atan2(y_dot,x_dot);
    
    % Initial Conditions
    x_robot(1) = x_0; % x-coord of the robot
    y_robot(1) = y_0; % y-coord of the robot
    theta_robot(1) = theta_0; % orientation of the robot
    
    % Loop di controllo
    for t=2:T_sim/dt
    
        % Desired linear and angular velocities
        v_d(t) = sqrt(x_dot(t)^2 + y_dot(t)^2);
        omega_d(t) = ( y_ddot(t)*x_dot(t) - x_ddot(t)*y_dot(t) ) / ( x_dot(t)^2 + y_dot(t)^2 );
    
        % Tracking Errors
        e1(t) = cos(theta_robot(t-1)) * (x_d(t) - x_robot(t-1)) + sin(theta_robot(t-1)) * (y_d(t) - y_robot(t-1));
        e2(t) = cos(theta_robot(t-1)) * (y_d(t) - y_robot(t-1)) - sin(theta_robot(t-1)) * (x_d(t) - x_robot(t-1));
        e3(t) = theta_d(t) - theta_robot(t-1);
        
        % Virtual Inputs
        u1 = -k1*e1(t);
        u2 = -k2*e2(t)-k3*e3(t);
    
        % Unicycle commands
        v(t) = v_d(t) * cos(e3(t)) - u1;
        omega(t) = omega_d(t) - u2;
       
        % Euler-update 
        % (Kinematic Model with perturbation d1,d2)
        theta_robot(t) = real(theta_robot(t-1) + (1+d2(t)) * omega(t) * dt);
        x_robot(t) = real(x_robot(t-1) + (1+d1(t))*cos(theta_robot(t-1)) * v(t) * dt);
        y_robot(t) = real(y_robot(t-1) + (1+d1(t))*sin(theta_robot(t-1)) * v(t) * dt);
    
        % Errors
        e(1,t)= real(x_robot(t) - x_d(t));
        e(2,t)= real(y_robot (t) - y_d(t));
        e(3,t)= real(theta_robot(t) - theta_d(t));
    
    end
end

