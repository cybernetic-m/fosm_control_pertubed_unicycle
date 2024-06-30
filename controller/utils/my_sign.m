%% FUNCTION TO COMPUTE THE SIGN FUNCTION

% Title: FOSM-AL Controls comparison of a perturbed Unicycle model
% Author: Massimo Romano

% This function simply compute the sign of an input x using 
% the (Shtessel et al., 2014) approach as an approximation

function x_sign = my_sign(x,beta)   
    x_sign = x/(abs(x) + beta);
end
