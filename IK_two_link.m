function [sol_1, sol_2] = IK_two_link( X, L1, L2 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%Solution_1
% PLANAR ROBOT
% D = (X(1)^2 + X(2)^2 - L1^2 - L2^2)/(2*L1*L2);
% 
% theta_2_sol_1 = atan2(sqrt(1-D^2),D);
% theta_2_sol_2 = atan2(-sqrt(1-D^2),D);
% 
% theta_1_sol_1 = atan2(X(2),X(1)) - atan2(L2*sin(theta_2_sol_1), L1+L2*cos(theta_2_sol_1));
% theta_1_sol_2 = atan2(X(2),X(1)) - atan2(L2*sin(theta_2_sol_2), L1+L2*cos(theta_2_sol_2));

% KUKA FIRST TWO LINKS

D = (X(3)-L1)/L2;

theta_2_sol_1 = atan2(D,sqrt(1-D^2));
theta_2_sol_2 = atan2(D,-sqrt(1-D^2));

% cos_11 = X(1)/(L2*cos(theta_2_sol_1));
% sin_11 = X(2)/(L2*cos(theta_2_sol_1));
% 
% cos_12 = X(1)/(L2*cos(theta_2_sol_2));
% sin_12 = X(2)/(L2*cos(theta_2_sol_2));

theta_1_sol_1 = atan2(X(2),X(1));
theta_1_sol_2 = theta_1_sol_1;

if theta_2_sol_1 >=0
    sol_1 = theta_1_sol_1;
    sol_2 = theta_2_sol_1;

else
    sol_1 = theta_1_sol_2;
    sol_2 = theta_2_sol_2;

end

end

