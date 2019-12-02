syms t q1(t) q2(t) x(t) y(t) z(t);
T = pi;
%T = 2*pi;
a1 = 0.38;
a2 = 0.24;

%Sampling trajectory
t = 0:0.025:T;

[q1, q2, dq1, dq2, ddq1, ddq2, dddq1, dddq2] = trajectory_gen_proj(a1,a2,t);

%Interpolation Matrices
poly_1 = zeros(length(q1),8);  %coefficients for joint 1
poly_2 = zeros(length(q1),8);  %coefficients for joint 2


for i = 1:length(q1)-1
    f = i+1;
    %Interpolation Matrix
    M = [1   t(i)  t(i)^2   t(i)^3     t(i)^4     t(i)^5      t(i)^6      t(i)^7;
         0    1    2*t(i)  3*t(i)^2   4*t(i)^3   5*t(i)^4    6*t(i)^5   7*t(i)^6;
         0    0      2      6*t(i)   12*t(i)^2   20*t(i)^3  30*t(i)^4   42*t(i)^5;
         0    0      0        6        24*t(i)   60*t(i)^2  120*t(i)^3  210*t(i)^4;
         1   t(f)  t(f)^2   t(f)^3     t(f)^4     t(f)^5      t(f)^6      t(f)^7;
         0    1    2*t(f)  3*t(f)^2   4*t(f)^3   5*t(f)^4    6*t(f)^5   7*t(f)^6;
         0    0      2      6*t(f)   12*t(f)^2   20*t(f)^3  30*t(f)^4   42*t(f)^5;
         0    0      0        6        24*t(f)   60*t(f)^2  120*t(f)^3  210*t(f)^4];

    %Interpolation for q1
    state_1 = [q1(i); dq1(i); ddq1(i); dddq1(i); q1(f); dq1(f); ddq1(f); dddq1(f)]; %INITIALIZING BEFORE WE HAVE VALUES!!!!
    %state_1 = [q1(i); dq1(i); ddq1(i); q1(f); dq1(f); ddq1(f)]; %INITIALIZING BEFORE WE HAVE VALUES!!!!
    A1 = pinv(M)*state_1;
    poly_1(i,:) = A1;
    %Interpolation for q2
    state_2 = [q2(i); dq2(i); ddq2(i); dddq2(i); q2(f); dq2(f); ddq2(f); dddq2(f)];
    %state_2 = [q2(i); dq2(i); ddq2(i); q2(f); dq2(f); ddq2(f)];
    A2 = pinv(M)*state_2;
    poly_2(i,:) = A2;
end

tspan = [0 T];
k=10000;
wn = 200;
damp = 0.98;
gamma = k;
% q_d = [pi/4; pi/4];
%y0 = [q1(1) q2(1) dq1(1) dq2(1)];
%y0 = [0 0 0 0];
y0 = [q1(1) q2(1) dq1(1) dq2(1) 1 1 0.5 1.5 0.1 0.5 0.5 0.5];
%[time,y] = ode45(@(time,y)pd_controller_612_proj(time,y,poly_1,poly_2,t,wn,damp),tspan,y0);
%[time,y] = ode45(@(time,y)pd_regulation_612_proj(time,y,q_d,k,k),tspan,y0);
[time,y] = ode45(@(time,y)adaptive_controller_612_proj(time,y,poly_1,poly_2,t,a2, gamma),tspan,y0);

[q1_d, q2_d, dq1_d, dq2_d, ddq1_d, ddq2_d, dddq1_d, dddq2_d] = trajectory_gen_proj(a1,a2,time);

s1 = (y(:,3)-dq1_d) + (y(:,1)-q1_d);
s2 = (y(:,4)-dq2_d) + (y(:,2)-q2_d);

%Plot Results

%Error
figure
plot(time,s1)
hold on
plot(time,s2)
xlabel('t')
ylabel('s')
title('Composite Error')
legend('s1','s2')


figure
plot(time,y(:,1))
hold on
scatter(t,q1)
xlabel('t')
ylabel('rad')
title('q1')
legend('Simulated', 'Desired','Location','southwest')
% 
figure
plot(time,y(:,3))
hold on
scatter(t,dq1)
axis([0,3.12,0,1.5])
xlabel('t')
ylabel('rad/s')
title('dq1')
legend('Simulated', 'Desired','Location','southwest')

figure
plot(time,y(:,2))
hold on
scatter(t,q2)
xlabel('t')
ylabel('rad')
title('q2')
legend('Simulated', 'Desired','Location','southwest')
% 
figure
plot(time,y(:,4))
hold on
scatter(t,dq2)
xlabel('t')
ylabel('rad/s')
title('dq2')
legend('Simulated', 'Desired','Location','southwest')


%Plot Forward Kinematics
cart_2 = [a2*cos(y(:,1)).*cos(y(:,2)), a2*sin(y(:,1)).*cos(y(:,2)), a2*sin(y(:,2))+a1];
figure
scatter3(cart_2(:,1),cart_2(:,2),cart_2(:,3))
hold on
[x,ys,z] = sphere;
mesh(a2*x,a2*ys,a2*z+a1) % centered at (3,-2,0)
axis([0,0.24,0, 0.24, 0, 0.7])
xlabel('x')
ylabel('y')
zlabel('z')
title('Cartesian')
% 
% 
% 
% % y0 = [0.2 0.3 0.1 0.1];
% % [time,y] = ode45(@(time,y)pd_controller_612(time,y,poly_1,poly_2,t,k),tspan,y0);
% % cart_2 = [a1*cos(y(:,1)) + a2*cos(y(:,1)+y(:,2)), a1*sin(y(:,1)) + a2*sin(y(:,1)+y(:,2))];
% % figure
% % scatter(cart_2(:,1),cart_2(:,2))
% % xlabel('x')
% % ylabel('y')
% % title('Convergence')
% 
%plot parameters 
figure
plot(time,y(:,5))
hold on
plot(time,y(:,6))
hold on
plot(time,y(:,7))
hold on
plot(time,y(:,8))
hold on
plot(time,y(:,9))
hold on
plot(time,y(:,10))
hold on
plot(time,y(:,11))
hold on
plot(time,y(:,12))


xlabel('t')
title('Parameters')
legend('J1', 'J2', 'J3', 'I_2y', 'I_1z', 'J4', 'J5', 'J6')
% 
% 
% 
% 
% 
