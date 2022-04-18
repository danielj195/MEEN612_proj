classdef adaptive_controller < controller
    %ADAPTIVE_CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lambda = eye(2);
        mu     = 800*eye(2);
        g      = 9.81;
        gamma  = 10000;  %adaptive gain

        %Physical constants(estimates)
        J1_star = 2;
        J2_star = 2;
        J3_star = 1;
        I_2y_star = 3;
        I_1z_star = 0.2;
        J4_star = 1;
        J5_star = 0.5;
        J6_star = 0.25;
    end
    
    methods
        function obj = adaptive_controller(polynomial_1, polynomial_2, time_vec)
            %ADAPTIVE_CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@controller(polynomial_1, polynomial_2, time_vec);
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
        
        function [dy] = adaptive_control_law(obj, t, y)
            %METHOD1 Control law for adaptive controller

            %State variables
            obj.q(1) = y(1);
            obj.q(2) = y(2);
            obj.dq(1) = y(3);
            obj.dq(2) = y(4);


            %Parameters
            J1 = y(5);
            J2 = y(6);
            J3 = y(7);
            I_2y = y(8);
            I_1z = y(9);
            J4 = y(10);
            J5 = y(11);
            J6 = y(12);

            [obj.coeff_1, obj.coeff_2] = find_poly_coeff(obj, t);
            [obj.q_des, obj.q_d_des, obj.q_dd_des] = find_des_state(obj, t);
            [obj.err, obj.d_err] = compute_error(obj);


            qr_dot = obj.q_d_des - obj.lambda*obj.err';
            qr_dot_dot = obj.q_dd_des - obj.lambda*obj.d_err';
            s = obj.dq' - qr_dot;
            
            J1_tilde = J1 - obj.J1_star;
            J2_tilde = J2 - obj.J2_star;
            J3_tilde = J3 - obj.J3_star;
            I_2y_tilde = I_2y - obj.I_2y_star;
            I_1z_tilde = I_1z - obj.I_1z_star;
            J4_tilde = J4 - obj.J4_star;
            J5_tilde = J5 - obj.J5_star;
            J6_tilde = J6 - obj.J6_star;
            
            
            %Ideal/actual mass and corialis matrices
            m11 = obj.J1_star*sin(obj.q(2))^2 + obj.I_2y_star*cos(obj.q(2))^2 + obj.I_1z_star;
            M_star = [ m11  0;
                       0  obj.J2_star];
                   
            c11 = obj.J1_star*sin(obj.q(2))*cos(obj.q(2))*obj.dq(2) - obj.I_2y_star*sin(obj.q(2))*cos(obj.q(2))*obj.dq(2);
            c12 = obj.J1_star*sin(obj.q(2))*cos(obj.q(2))*obj.dq(1) - obj.I_2y_star*sin(obj.q(2))*cos(obj.q(2))*obj.dq(1);
            c21 = -c12;
            c22 = 0;
            C_star = [ c11  c12;
                       c21  c22];
              
            %Regressors
            % Y1 -> J1;  Y2 -> J2;  Y3 -> J3; Y4  -> I_2Y;  Y5 -> I_1Z;  Y6 -> J4; 
            % Y7 -> J5;  Y8 -> J6
            Y1 = [sin(obj.q(2))^2*qr_dot_dot(1) + sin(obj.q(2))*cos(obj.q(2))*obj.dq(2)*qr_dot(2);
                 -sin(obj.q(2))*cos(obj.q(2))*obj.dq(1)*qr_dot(1)];
            Y2 = [0; qr_dot_dot(2)];
            Y3 = [0; -sin(obj.q(2))*obj.g];
            Y4 = [cos(obj.q(2))^2*qr_dot_dot(1) - sin(obj.q(2))*cos(obj.q(2))*obj.dq(2)*qr_dot(1) - sin(obj.q(2))*cos(obj.q(2))*obj.dq(1)*qr_dot(2);
                  sin(obj.q(2))*cos(obj.q(2))*obj.dq(1)*qr_dot(1)];
            Y5 = [qr_dot_dot(1); 0];
            Y6 = [-sin(obj.q(1))*sin(obj.q(2)); cos(obj.q(1))*cos(obj.q(2))];
            Y7 = [cos(obj.q(1))*sin(obj.q(2)); sin(obj.q(1))*cos(obj.q(2))];
            Y8 = [0; -sin(obj.q(2))];
            
                
            q_dot_dot = M_star\(C_star*s + J1_tilde*Y1 + J2_tilde*Y2 + J3_tilde*Y3 ...
                + I_2y_tilde*Y4 + I_1z_tilde*Y5 + J4_tilde*Y6 + J5_tilde*Y7 +J6_tilde*Y8 - obj.mu*s)...
                + obj.q_dd_des - obj.lambda*obj.d_err';
            
            dy = zeros(12,1);
            
            
            dy(1) = obj.dq(1);
            dy(2) = obj.dq(2);
            dy(3) = q_dot_dot(1);
            dy(4) = q_dot_dot(2);
            dy(5) = (-1/obj.gamma)*s.'*Y1;
            dy(6) = (-1/obj.gamma)*s.'*Y2;
            dy(7) = (-1/obj.gamma)*s.'*Y3;
            dy(8) = (-1/obj.gamma)*s.'*Y4;
            dy(9) = (-1/obj.gamma)*s.'*Y5;
            dy(10) = (-1/obj.gamma)*s.'*Y6;
            dy(11) = (-1/obj.gamma)*s.'*Y7;
            dy(12) = (-1/obj.gamma)*s.'*Y8;
        end
    end
end

