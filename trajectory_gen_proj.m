function [q1, q2, dq1, dq2, ddq1, ddq2, dddq1, dddq2] = trajectory_gen_proj(a1,a2,time)
syms t q1(t) q2(t) x(t) y(t) z(t);
%Trajectory and derivatives
m = cot(1);
x(t) = a2*cos(t)/cosh(m*t);  %loxodrome
y(t) = a2*sin(t)/cosh(m*t);
z(t) = a1 + a2*tanh(m*t);
dx = diff(x,t);
dy = diff(y,t);
dz = diff(z,t);
ddx = diff(dx,t);
ddy = diff(dy,t);
ddz = diff(dz,t);
dddx = diff(ddx,t);
dddy = diff(ddy,t);
dddz = diff(ddz,t);

t = time;

%Substitute to obtain cartesian variables
xdb = double(subs(x));
ydb = double(subs(y));
zdb = double(subs(z));
dxdb = double(subs(dx));
dydb = double(subs(dy));
dzdb = double(subs(dz));
ddxdb = double(subs(ddx));
ddydb = double(subs(ddy));
ddzdb = double(subs(ddz));
dddxdb = double(subs(dddx));
dddydb = double(subs(dddy));
dddzdb = double(subs(dddz));

%Initialize joint space variables
q1 = zeros(length(xdb),1);
q2 = zeros(length(xdb),1);
dq1 = zeros(length(xdb),1);
dq2 = zeros(length(xdb),1);
ddq1 = zeros(length(xdb),1);
ddq2 = zeros(length(xdb),1);
dddq1 = zeros(length(xdb),1);
dddq2 = zeros(length(xdb),1);


%Inverse Kinematics to get joint angles
for i = 1:length(q1)
    [q1(i), q2(i)] = IK_two_link([xdb(i),ydb(i),zdb(i)],a1,a2);
    %Joint Velocity
    J = Jacobian_func([q1(i),q2(i)],a1,a2);
    dq = J\[dxdb(i); dydb(i); dzdb(i)];
    dq1(i) = dq(1);
    dq2(i) = dq(2);
    %Joint Acceleration
    dJ = dJacobian([q1(i),q2(i)],[dq1(i),dq2(i)],a1,a2);
    B = [ddxdb(i);ddydb(i);ddzdb(i)] - dJ*dq;
    ddq = J\B;
    ddq1(i) = ddq(1);
    ddq2(i) = ddq(2);
    %Joint Jerk
    ddJ = ddJacobian([q1(i),q2(i)],[dq1(i),dq2(i)],[ddq1(i),ddq2(i)],a1,a2);
    C = [dddxdb(i);dddydb(i);dddzdb(i)] - (ddJ*dq + 2*dJ*ddq);
    dddq = J\C;
    dddq1(i) = dddq(1);
    dddq2(i) = dddq(2);
end
end

