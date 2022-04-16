classdef trajectory
    %TRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
   
    properties (Access = protected)
        time;
        time_vec;
        x, y, z;
        L1, L2;
    end
    
    methods
        function obj = trajectory(time_vec, L1, L2)
            %TRAJECTORY Construct an instance of this class
            %   Detailed explanation goes here
           
            obj.time_vec = time_vec;
            obj.L1 = L1;
            obj.L2 = L2;
        end


        function [xc, yc, zc] = compose_func(obj)
            syms t_s
            S = obj.time_vec(end)/(1+(exp(-3*(t_s-2))));  %Sigmoid for time Scaling
            St = compose(S, obj.time);
            xc = compose(obj.x, St);
            yc = compose(obj.y, St);
            zc = compose(obj.z, St);
        end

        function [dx, dy, dz, ddx, ddy, ddz, dddx, dddy, dddz] = differentiate(obj)
            %METHOD1 Differentiate trajectory
            [xc, yc, zc] = compose_func(obj);
            dx = diff(xc,obj.time);
            dy = diff(yc,obj.time);
            dz = diff(zc,obj.time);
            ddx = diff(dx,obj.time);
            ddy = diff(dy,obj.time);
            ddz = diff(dz,obj.time);
            dddx = diff(ddx,obj.time);
            dddy = diff(ddy,obj.time);
            dddz = diff(ddz,obj.time);
        end

        function [xdb, ydb, zdb, dxdb, dydb, dzdb, ddxdb, ddydb, ddzdb, dddxdb, dddydb, dddzdb] = substitute(obj)
            %METHOD2 Substitute discrete data into symbolic variables
            [dx, dy, dz, ddx, ddy, ddz, dddx, dddy, dddz] = ...
                differentiate(obj);
            [xc, yc, zc] = compose_func(obj);
            t = obj.time_vec;
            %Substitute to obtain cartesian variables
            xdb = double(subs(xc));
            ydb = double(subs(yc));
            zdb = double(subs(zc));
            dxdb = double(subs(dx));
            dydb = double(subs(dy));
            dzdb = double(subs(dz));
            ddxdb = double(subs(ddx));
            ddydb = double(subs(ddy));
            ddzdb = double(subs(ddz));
            dddxdb = double(subs(dddx));
            dddydb = double(subs(dddy));
            dddzdb = double(subs(dddz));
        end

        function [q1, q2, dq1, dq2, ddq1, ddq2, dddq1, dddq2] = generate_traj(obj)
            [xdb, ydb, zdb, dxdb, dydb, dzdb, ddxdb, ddydb, ddzdb,...
                dddxdb, dddydb, dddzdb] = substitute(obj);
            %METHOD3 Generate trajectory

            kin = robot_kin(obj.L1, obj.L2);
           
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
                X = [xdb(i), ydb(i), zdb(i)];
                [q1(i), q2(i)] = kin.inv_kin(X);

                %Joint Velocity
                J = kin.jacobian([q1(i),q2(i)]);
                dq = J\[dxdb(i); dydb(i); dzdb(i)];
                dq1(i) = dq(1);
                dq2(i) = dq(2);

                %Joint Acceleration
                dJ = kin.djacobian([q1(i),q2(i)],[dq1(i),dq2(i)]);
                B = [ddxdb(i);ddydb(i);ddzdb(i)] - dJ*dq;
                ddq = J\B;
                ddq1(i) = ddq(1);
                ddq2(i) = ddq(2);

                %Joint Jerk
                ddJ = kin.ddjacobian([q1(i),q2(i)],[dq1(i),dq2(i)],[ddq1(i),ddq2(i)]);
                C = [dddxdb(i);dddydb(i);dddzdb(i)] - (ddJ*dq + 2*dJ*ddq);
                dddq = J\C;
                dddq1(i) = dddq(1);
                dddq2(i) = dddq(2);
            end

        end

        function [poly_1, poly_2] = interpolate_traj(obj, t, q1, q2, dq1, dq2, ddq1, ddq2, dddq1, dddq2)
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
                state_1 = [q1(i); dq1(i); ddq1(i); dddq1(i); q1(f); dq1(f); ddq1(f); dddq1(f)];
                A1 = pinv(M)*state_1;
                poly_1(i,:) = A1;
                %Interpolation for q2
                state_2 = [q2(i); dq2(i); ddq2(i); dddq2(i); q2(f); dq2(f); ddq2(f); dddq2(f)];
                A2 = pinv(M)*state_2;
                poly_2(i,:) = A2;
            end 
        end


    end
end

