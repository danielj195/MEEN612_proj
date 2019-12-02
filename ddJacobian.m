function [ ddJ ] = ddJacobian( q, dq, ddq, L1, L2 )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%PLANAR ROBOT
% j11 = -L1*(-sin(q(1))*dq(1)^2 + cos(q(1))*ddq(1)) - ...
%     L2*(-sin(q(1)+q(2))*(dq(1)+dq(2))^2 + cos(q(1)+q(2))*(ddq(1)+ddq(2)));
% j12 = -L2*(-sin(q(1)+q(2))*(dq(1)+dq(2))^2 + cos(q(1)+q(2))*(ddq(1)+ddq(2)));
% 
% j21 = -L1*(-cos(q(1))*dq(1)^2 + sin(q(1))*ddq(1)) - ...
%     L2*(cos(q(1)+q(2))*(dq(1)+dq(2))^2 + sin(q(1)+q(2))*(ddq(1)+ddq(2)));
% j22 = -L2*(cos(q(1)+q(2))*(dq(1)+dq(2))^2 + sin(q(1)+q(2))*(ddq(1)+ddq(2)));

%KUKA 2 LINK 
j11 = -L2*((-cos(q(2))*dq(2)^2*sin(q(1)) - sin(q(2))*ddq(2)*sin(q(1)) - sin(q(2))*dq(2)*cos(q(1))*dq(1))+...
    (-sin(q(2))*dq(2)*cos(q(1))*dq(1) - cos(q(2))*sin(q(1))*dq(1)^2 + cos(q(2))*cos(q(1))*ddq(1)));

j12 = -L2*((-cos(q(1))*dq(1)^2*sin(q(2)) - sin(q(1))*ddq(1)*sin(q(2)) - sin(q(1))*dq(1)*cos(q(2))*dq(2))+...
    (-sin(q(1))*dq(1)*cos(q(2))*dq(2) - cos(q(1))*sin(q(2))*dq(2)^2 + cos(q(1))*cos(q(2))*ddq(2)));

j21 = L2*((-cos(q(2))*dq(2)^2*cos(q(1)) - sin(q(2))*ddq(2)*cos(q(1)) + sin(q(2))*dq(2)*sin(q(1))*dq(1))+...
    (sin(q(2))*dq(2)*sin(q(1))*dq(1) - cos(q(2))*cos(q(1))*dq(1)^2 + cos(q(2))*sin(q(1))*ddq(1)));

j22 = -L2*((-sin(q(1))*dq(1)^2*sin(q(2)) + cos(q(1))*ddq(1)*sin(q(2)) + cos(q(1))*dq(1)*cos(q(2))*dq(2))+...
    (cos(q(1))*dq(1)*cos(q(2))*dq(2) - sin(q(1))*sin(q(2))*dq(2)^2 + sin(q(1))*cos(q(2))*ddq(2)));

j31 = 0;

j32 = -L2*(cos(q(2))*dq(2)^2 + sin(q(2))*ddq(2));

ddJ = [j11 j12; j21 j22; j31 j32];
end

