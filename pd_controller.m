classdef pd_controller < controller
    %PD_CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        wn = 200;
        damp = 0.98;
        Kp = 0;
        Kd = 0;
    end
    
    methods
        function obj = pd_controller(polynomial_1, polynomial_2, time_vec)
            %ADAPTIVE_CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@controller(polynomial_1, polynomial_2, time_vec);
            obj.Kp = obj.wn^2*eye(2);
            obj.Kd = 2*obj.damp*obj.wn*eye(2);
        end

        %Inherited Methods:

        function [coeff_1, coeff_2] = find_poly_coeff(obj,t)
            [coeff_1, coeff_2] = find_poly_coeff@controller(obj,t);
        end


        function [q_des, q_d_des, q_dd_des] = find_des_state(obj, t)
            [q_des, q_d_des, q_dd_des] = find_des_state@controller(obj, t);
        end

        function [err, d_err] = compute_error(obj)
            [err, d_err] = compute_error@controller(obj);
        end




        %Derived Methods
        
        function [dy] = pd_control_law(obj, t, y)
            %METHOD1 Control law for adaptive controller

            %State variables
            obj.q(1) = y(1);
            obj.q(2) = y(2);
            obj.dq(1) = y(3);
            obj.dq(2) = y(4);


            [obj.coeff_1, obj.coeff_2] = find_poly_coeff(obj, t);
            [obj.q_des, obj.q_d_des, obj.q_dd_des] = find_des_state(obj, t);
            [obj.err, obj.d_err] = compute_error(obj);
            q_dot = obj.q_dd_des - obj.Kd*obj.d_err' - obj.Kp*obj.err';
               
            dy = zeros(4,1);
            
            dy(1) = obj.dq(1);
            dy(2) = obj.dq(2);
            dy(3) = q_dot(1);
            dy(4) = q_dot(2);

        end
    end
end

