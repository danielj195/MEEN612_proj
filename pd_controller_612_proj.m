function [dy] = pd_controller_612_proj(t,y,poly_1,poly_2,T,wn,damp)
    %Inputs: t: time   
    %q1: vector of joint positions
    %q2: vector of joint velocities
    interval = find(T<=t);
    func_ind = interval(end)-1;
    if func_ind < 1
        func_ind = 1;
    end
    A1 = poly_1(func_ind,:);
    A2 = poly_2(func_ind,:);
    q1_pos = y(1);
    if q1_pos > pi
        q1_pos = q1_pos - 2*pi;
    end
    q2_pos = y(2);
     if q2_pos > pi
        q2_pos = q2_pos - 2*pi;
    end
    q1_vel = y(3);
    q2_vel = y(4);
    
    q_des = [A1(1) + A1(2)*t + A1(3)*t^2 + A1(4)*t^3 + A1(5)*t^4 + A1(6)*t^5 + A1(7)*t^6 + A1(8)*t^7;
             A2(1) + A2(2)*t + A2(3)*t^2 + A2(4)*t^3 + A2(5)*t^4 + A2(6)*t^5 + A2(7)*t^6 + A2(8)*t^7];
     
    q_dot_des = [A1(2) + 2*A1(3)*t + 3*A1(4)*t^2 + 4*A1(5)*t^3 + 5*A1(6)*t^4 + 6*A1(7)*t^5 + 7*A1(8)*t^6;
                 A2(2) + 2*A2(3)*t + 3*A2(4)*t^2 + 4*A2(5)*t^3 + 5*A2(6)*t^4 + 6*A2(7)*t^5 + 7*A2(8)*t^6];
             
    q_dot_dot_des = [2*A1(3) + 6*A1(4)*t + 12*A1(5)*t^2 + 20*A1(6)*t^3 + 30*A1(7)*t^4 + 42*A1(8)*t^5;
                     2*A2(3) + 6*A2(4)*t + 12*A2(5)*t^2 + 20*A2(6)*t^3 + 30*A2(7)*t^4 + 42*A2(8)*t^5];
    
%     q_des = [0.1*cos(pi*t/2); 0.1*sin(pi*t/2)];
%     q_dot_des = [(-0.1*pi/2)*sin(pi*t/2); (0.1*pi/2)*cos(pi*t/2)];
%     q_dot_dot_des = [(-0.1*pi^2/4)*cos(pi*t/2); (-0.1*pi^2/4)*sin(pi*t/2)];
    
    Kp = wn^2*eye(2);
    Kd = 2*damp*wn*eye(2);

    err = [q1_pos; q2_pos] - q_des;
    d_err = [q1_vel; q2_vel] - q_dot_des;

    q_dot = q_dot_dot_des - Kd*d_err - Kp*err;
    dy = zeros(4,1);
    dy(1) = q1_vel;
    dy(2) = q2_vel;
    dy(3) = q_dot(1);
    dy(4) = q_dot(2);
end

