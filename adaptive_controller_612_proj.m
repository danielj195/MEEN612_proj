function [dy] = adaptive_controller_612_proj(t,y,poly_1,poly_2,T,L2,gamma)
%Input: t-time    y-state variables   p-parameters

interval = find(T<=t);
func_ind = interval(end)-1;
if func_ind < 1
    func_ind = 1;
end
A1 = poly_1(func_ind,:);
A2 = poly_2(func_ind,:);

%Controller and Adaptive gains
lambda = eye(2);
mu = 800*eye(2);
g = 9.81;

%State variables
q1_pos = y(1);
q2_pos = y(2);
q1_vel = y(3);
q2_vel = y(4);
J1 = y(5);
J2 = y(6);
J3 = y(7);
I_2y = y(8);
I_1z = y(9);
J4 = y(10);
J5 = y(11);
J6 = y(12);

% q_des = [0.1*cos(pi*t/2); 0.1*sin(pi*t/2)];
% q_dot_des = [(-0.1*pi/2)*sin(pi*t/2); (0.1*pi/2)*cos(pi*t/2)];
% q_dot_dot_des = [(-0.1*pi^2/4)*cos(pi*t/2); (-0.1*pi^2/4)*sin(pi*t/2)];

q_des = [A1(1) + A1(2)*t + A1(3)*t^2 + A1(4)*t^3 + A1(5)*t^4 + A1(6)*t^5 + A1(7)*t^6 + A1(8)*t^7;
         A2(1) + A2(2)*t + A2(3)*t^2 + A2(4)*t^3 + A2(5)*t^4 + A2(6)*t^5 + A2(7)*t^6 + A2(8)*t^7];
     
q_dot_des = [A1(2) + 2*A1(3)*t + 3*A1(4)*t^2 + 4*A1(5)*t^3 + 5*A1(6)*t^4 + 6*A1(7)*t^5 + 7*A1(8)*t^6;
             A2(2) + 2*A2(3)*t + 3*A2(4)*t^2 + 4*A2(5)*t^3 + 5*A2(6)*t^4 + 6*A2(7)*t^5 + 7*A2(8)*t^6];
         
q_dot_dot_des = [2*A1(3) + 6*A1(4)*t + 12*A1(5)*t^2 + 20*A1(6)*t^3 + 30*A1(7)*t^4 + 42*A1(8)*t^5;
                 2*A2(3) + 6*A2(4)*t + 12*A2(5)*t^2 + 20*A2(6)*t^3 + 30*A2(7)*t^4 + 42*A2(8)*t^5];

err = [q1_pos; q2_pos] - q_des;
err_dot = [q1_vel; q2_vel] - q_dot_des;
qr_dot = q_dot_des - lambda*err;
qr_dot_dot = q_dot_dot_des - lambda*err_dot;
s = [q1_vel; q2_vel] - qr_dot;



%Physical constants(known)
J1_star = 2;
J2_star = 2;
J3_star = 1;
I_2y_star = 3;
I_1z_star = 0.2;
J4_star = 1;
J5_star = 0.5;
J6_star = 0.25;

J1_tilde = J1 - J1_star;
J2_tilde = J2 - J2_star;
J3_tilde = J3 - J3_star;
I_2y_tilde = I_2y - I_2y_star;
I_1z_tilde = I_1z - I_1z_star;
J4_tilde = J4 - J4_star;
J5_tilde = J5 - J5_star;
J6_tilde = J6 - J6_star;


%Ideal/actual mass and corialis matrices
m11 = J1_star*sin(q2_pos)^2 + I_2y_star*cos(q2_pos)^2 + I_1z_star;
M_star = [ m11  0;
           0  J2_star];
       
c11 = J1_star*sin(q2_pos)*cos(q2_pos)*q2_vel - I_2y_star*sin(q2_pos)*cos(q2_pos)*q2_vel;
c12 = J1_star*sin(q2_pos)*cos(q2_pos)*q1_vel - I_2y_star*sin(q2_pos)*cos(q2_pos)*q1_vel;
c21 = -c12;
c22 = 0;
C_star = [ c11  c12;
           c21  c22];
  
%Regressors
% Y1 -> J1;  Y2 -> J2;  Y3 -> J3; Y4  -> I_2Y;  Y5 -> I_1Z;  Y6 -> J4; 
% Y7 -> J5;  Y8 -> J6
Y1 = [sin(q2_pos)^2*qr_dot_dot(1) + sin(q2_pos)*cos(q2_pos)*q2_vel*qr_dot(2);
     -sin(q2_pos)*cos(q2_pos)*q1_vel*qr_dot(1)];
Y2 = [0; qr_dot_dot(2)];
Y3 = [0; -sin(q2_pos)*g];
Y4 = [cos(q2_pos)^2*qr_dot_dot(1) - sin(q2_pos)*cos(q2_pos)*q2_vel*qr_dot(1) - sin(q2_pos)*cos(q2_pos)*q1_vel*qr_dot(2);
      sin(q2_pos)*cos(q2_pos)*q1_vel*qr_dot(1)];
Y5 = [qr_dot_dot(1); 0];
Y6 = [-sin(q1_pos)*sin(q2_pos); cos(q1_pos)*cos(q2_pos)];
Y7 = [cos(q1_pos)*sin(q2_pos); sin(q1_pos)*cos(q2_pos)];
Y8 = [0; -sin(q2_pos)];

    
q_dot_dot = M_star\(C_star*s + J1_tilde*Y1 + J2_tilde*Y2 + J3_tilde*Y3 ...
    + I_2y_tilde*Y4 + I_1z_tilde*Y5 + J4_tilde*Y6 + J5_tilde*Y7 +J6_tilde*Y8 - mu*s)...
    + q_dot_dot_des - lambda*err_dot;

dy = zeros(12,1);


dy(1) = q1_vel;
dy(2) = q2_vel;
dy(3) = q_dot_dot(1);
dy(4) = q_dot_dot(2);
dy(5) = (-1/gamma)*s.'*Y1;
dy(6) = (-1/gamma)*s.'*Y2;
dy(7) = (-1/gamma)*s.'*Y3;
dy(8) = (-1/gamma)*s.'*Y4;
dy(9) = (-1/gamma)*s.'*Y5;
dy(10) = (-1/gamma)*s.'*Y6;
dy(11) = (-1/gamma)*s.'*Y7;
dy(12) = (-1/gamma)*s.'*Y8;


end

