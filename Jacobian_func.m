function [ J ] = Jacobian_func(q, L1, L2)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% PLANAR ROBOT
% J = [-L1*sin(q(1)) - L2*sin(q(1)+q(2))   -L2*sin(q(1)+q(2));
%       L1*cos(q(1)) + L2*cos(q(1)+q(2))    L2*cos(q(1)+q(2))];

%KUKA FIRST 2 LINKS
J = [-L2*cos(q(2))*sin(q(1)) -L2*cos(q(1))*sin(q(2));
     L2*cos(q(2))*cos(q(1)) -L2*sin(q(1))*sin(q(2));
     0                        L2*cos(q(2))          ];
end

