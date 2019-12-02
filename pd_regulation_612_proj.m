function [dy] = pd_regulation_612_proj(t,y,q_d,kp,kd)
    %Inputs: t: time   
    %q1: vector of joint positions
    %q2: vector of joint velocities

    q1_pos = y(1);
    q2_pos = y(2);
    q1_vel = y(3);
    q2_vel = y(4);
    
    err = [q1_pos; q2_pos] - q_d;
    d_err = [q1_vel; q2_vel];
    
    Kp = kp*eye(2);
    Kd = kd*eye(2);
    %Physical constants(known)
    J1_star = 2;
    J2_star = 2;
    I_2y_star = 3;
    I_1z_star = 0.2;

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
           
    B = -C_star*d_err - Kd*d_err - Kp*err;

    q_dot = M_star\B;
    dy = zeros(4,1);
    dy(1) = q1_vel;
    dy(2) = q2_vel;
    dy(3) = q_dot(1);
    dy(4) = q_dot(2);
end

