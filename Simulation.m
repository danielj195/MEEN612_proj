classdef Simulation
    %SIMULATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        trajectory;
        controller;
        traj_options;
        ctrllr_options;
        L1;
        L2;
        T = pi;
        t_des;
    end
    
    methods
        function obj = Simulation(traj, ctrllr, time)
            %SIMULATION Construct an instance of this class
            obj.trajectory = traj;
            obj.controller = ctrllr;
            obj.traj_options = ["loxodrome", "sinusoid"];
            obj.ctrllr_options = ["pd", "adaptive"];
            obj.L1 = 0.38;
            obj.L2 = 0.24;
%             obj.L1 = robot(1);
%             obj.L2 = robot(2);
            obj.T = time;
            simulate(obj);
        end
        
        function [] = simulate(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            t = 0:0.025:obj.T;
            obj.t_des = t;
            tspan = [0 obj.T];
            %Choose Trajectory
            if obj.trajectory == obj.traj_options(1)
                loxo = Loxodrome(t,obj.L1,obj.L2);
                [q1, q2, dq1, dq2, ddq1, ddq2, dddq1, dddq2] = loxo.generate_traj();
                [poly_1, poly_2] = loxo.interpolate_traj(t,q1, q2, dq1,...
                    dq2, ddq1, ddq2, dddq1, dddq2);
            elseif obj.trajectory == obj.traj_options(2)
            end

            %Choose Controller
             if obj.controller == obj.ctrllr_options(1)
                y0 = [0 0 0 0];
                pd = pd_controller(poly_1, poly_2, t);
                [time,y] = ode45(@(time,y)pd.pd_control_law(time, y),tspan,y0);
            elseif obj.controller == obj.ctrllr_options(2)
                y0 = [q1(1) q2(1) dq1(1) dq2(1) 1 1 0.5 1.5 0.1 0.5 0.5 0.5];
                adapt = adaptive_controller(poly_1, poly_2, t);
                [time,y] = ode45(@(time,y)adapt.adaptive_control_law(time, y),tspan,y0);
            end

            plot_error(obj, time, y);
            plot_state(obj, time, y, q1, q2, dq1, dq2);
            plot_traj(obj, time, y);


        end

        function [] = plot_error(obj, time, y)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            if obj.trajectory == obj.traj_options(1)
                loxo = Loxodrome(time, obj.L1, obj.L2);
                [q1_d, q2_d, dq1_d, dq2_d, ddq1_d, ddq2_d, dddq1_d, dddq2_d] = loxo.generate_traj();
                s1 = (y(:,3)-dq1_d) + (y(:,1)-q1_d);
                s2 = (y(:,4)-dq2_d) + (y(:,2)-q2_d);
            elseif obj.trajectory == obj.traj_options(2)
            end

            figure
            plot(time,s1)
            hold on
            plot(time,s2)
            xlabel('t')
            ylabel('s')
            title('Composite Error')
            legend('s1','s2')
           
        end

        function [] = plot_state(obj, time, y, q1, q2, dq1, dq2)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            figure
            plot(time,y(:,1))
            hold on
            scatter(obj.t_des, q1)
            xlabel('t')
            ylabel('rad')
            title('q1')
            legend('Simulated', 'Desired','Location','southwest')
            % 
            figure
            plot(time,y(:,3))
            hold on
            scatter(obj.t_des,dq1)
            axis([0,3.12,0,1.5])
            xlabel('t')
            ylabel('rad/s')
            title('dq1')
            legend('Simulated', 'Desired','Location','southwest')
            
            figure
            plot(time,y(:,2))
            hold on
            scatter(obj.t_des,q2)
            xlabel('t')
            ylabel('rad')
            title('q2')
            legend('Simulated', 'Desired','Location','southwest')
            % 
            figure
            plot(time,y(:,4))
            hold on
            scatter(obj.t_des,dq2)
            xlabel('t')
            ylabel('rad/s')
            title('dq2')
            legend('Simulated', 'Desired','Location','southwest')                   
           
        end

        function [] = plot_traj(obj, time, y)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            robot = robot_kin(obj.L1, obj.L2);
            X = robot.forward_kin(y(:,1), y(:,2));

            figure
            scatter3(X(:,1), X(:,2), X(:,3))
            hold on
            [x,ys,z] = sphere;
            mesh(obj.L2*x, obj.L2*ys, obj.L2*z+obj.L1) % centered at (3,-2,0)
            axis([0,0.24,0, 0.24, 0, 0.7])
            xlabel('x')
            ylabel('y')
            zlabel('z')
            title('Cartesian')
           
        end

    end
end

