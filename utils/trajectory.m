%% FUNCTION TO GENERATE THE TRAJECTORY

% Title: FOSM-AL Controls comparison of a perturbed Unicycle model
% Author: Massimo Romano

% This function generate three types of trajectories:
% 1) Circle
% 2) Lemniscate (Infinity) Curve


function  [x_d, y_d, x_dot, y_dot, x_ddot, y_ddot, t] = trajectory (choice)
   
    n_points = 10000;    % Number of points
    
    switch choice
        case 'Circle'
            % Variables
            t = linspace(0 , 2*pi, n_points); 
            
            % Parametric Equation 
            x_d = cos(t);
            y_d = sin(t);
            
            % First derivatives of the trajectory
            x_dot = -sin(t);
            y_dot = cos(t);
            
            % Second derivatives of the trajectory
            x_ddot = -cos(t);
            y_ddot = -sin(t);

        case 'Lemniscate'
            % Variables
            t = linspace(0 , 3*pi, n_points); 
            
            % Parametric Equation 
            x_d = cos(t);
            y_d = sin(2*t);
            
            % First derivatives of the trajectory
            x_dot = -sin(t);
            y_dot = 2*cos(2*t);
            
            % Second derivatives of the trajectory
            x_ddot = -cos(t);
            y_ddot = -4*sin(2*t);

        otherwise
            error('Invalid choice');
    end

end