% syms t q1(t) q2(t) x(t) y(t) z(t);
T = pi;
%T = 2*pi;
a1 = 0.38;
a2 = 0.24;
max_vel = 0.6;

%Sampling trajectory
t = 0:0.025:T;

% [q1, q2, dq1, dq2, ddq1, ddq2, dddq1, dddq2] = trajectory_gen_proj(a1,a2,t);

loxo = Loxodrome(t,a1,a2);
[q1, q2, dq1, dq2, ddq1, ddq2, dddq1, dddq2] = loxo.generate_traj();
[poly_1, poly_2] = loxo.interpolate_traj(t,q1, q2, dq1, dq2, ddq1, ddq2, dddq1, dddq2);


tspan = [0 T];
k=10000;
wn = 200;
damp = 0.98;
gamma = k;
% q_d = [pi/4; pi/4];
%y0 = [q1(1) q2(1) dq1(1) dq2(1)];
y0 = [0 0 0 0];
%y0 = [q1(1) q2(1) dq1(1) dq2(1) 1 1 0.5 1.5 0.1 0.5 0.5 0.5];
%[time,y] = ode45(@(time,y)pd_controller_612_proj(time,y,poly_1,poly_2,t,wn,damp),tspan,y0);
%[time,y] = ode45(@(time,y)pd_regulation_612_proj(time,y,q_d,k,k),tspan,y0);
%[time,y] = ode45(@(time,y)adaptive_controller_612_proj(time,y,poly_1,poly_2,t,a2, gamma),tspan,y0);
adapt = adaptive_controller(poly_1, poly_2, t);
pd = pd_controller(poly_1, poly_2, t);
% [time,y] = ode45(@(time,y)adapt.adaptive_control_law(time, y),tspan,y0);
[time,y] = ode45(@(time,y)pd.pd_control_law(time, y),tspan,y0);


% [q1_d, q2_d, dq1_d, dq2_d, ddq1_d, ddq2_d, dddq1_d, dddq2_d] = trajectory_gen_proj(a1,a2,time);
loxo2 = Loxodrome(time,a1,a2);
[q1_d, q2_d, dq1_d, dq2_d, ddq1_d, ddq2_d, dddq1_d, dddq2_d] = loxo2.generate_traj();

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

