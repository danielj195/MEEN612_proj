classdef controller
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = protected)
        q_des     = [0; 0];
        q_d_des   = [0; 0];
        q_dd_des  = [0; 0];
        coeff_1   = zeros(1, 8);
        coeff_2   = zeros(1, 8);
        err       = [0; 0];
        d_err     = [0; 0];
        q;
        dq;
        %  Trajectory params not initialized because length unknown
        poly_1;
        poly_2;
        %T;
    end

    properties (Access = public)
       T; 
       num = 0;
    end
    
    methods
        function obj = controller(polynomial_1, polynomial_2, time_vec)
            %CONTROLLERS Construct an instance of this class
            %   Detailed explanation goes here
            obj.poly_1 = polynomial_1;
            obj.poly_2 = polynomial_2;
            obj.T      = time_vec;
        end
        
        function [coeff_1, coeff_2] = find_poly_coeff(obj, t)
            %METHOD1 Find the coefficients of the interpolating polynomial
            interval = find(obj.T<=t);
            func_ind = interval(end)-1;
            if func_ind < 1
                func_ind = 1;
            end
            obj.num = obj.num + 1;
            coeff_1 = obj.poly_1(func_ind,:);
            coeff_2 = obj.poly_2(func_ind,:);
        end


        function [q_des, q_d_des, q_dd_des] = find_des_state(obj, t)
            %METHOD2 Use the interpolating polynomial to find the desired
            %state
            q_des = [obj.coeff_1(1) + obj.coeff_1(2)*t + obj.coeff_1(3)*t^2 + obj.coeff_1(4)*t^3 + ...
                     obj.coeff_1(5)*t^4 + obj.coeff_1(6)*t^5 + obj.coeff_1(7)*t^6 + obj.coeff_1(8)*t^7;
                     obj.coeff_2(1) + obj.coeff_2(2)*t + obj.coeff_2(3)*t^2 + obj.coeff_2(4)*t^3 + ...
                     obj.coeff_2(5)*t^4 + obj.coeff_2(6)*t^5 + obj.coeff_2(7)*t^6 + obj.coeff_2(8)*t^7];
     
            q_d_des = [obj.coeff_1(2) + 2*obj.coeff_1(3)*t + 3*obj.coeff_1(4)*t^2 + 4*obj.coeff_1(5)*t^3 + ...
                       5*obj.coeff_1(6)*t^4 + 6*obj.coeff_1(7)*t^5 + 7*obj.coeff_1(8)*t^6;
                       obj.coeff_2(2) + 2*obj.coeff_2(3)*t + 3*obj.coeff_2(4)*t^2 + 4*obj.coeff_2(5)*t^3 + ...
                       5*obj.coeff_2(6)*t^4 + 6*obj.coeff_2(7)*t^5 + 7*obj.coeff_2(8)*t^6];
                     
            q_dd_des = [2*obj.coeff_1(3) + 6*obj.coeff_1(4)*t + 12*obj.coeff_1(5)*t^2 + ...
                        20*obj.coeff_1(6)*t^3 + 30*obj.coeff_1(7)*t^4 + 42*obj.coeff_1(8)*t^5;
                        2*obj.coeff_2(3) + 6*obj.coeff_2(4)*t + 12*obj.coeff_2(5)*t^2 + ...
                        20*obj.coeff_2(6)*t^3 + 30*obj.coeff_2(7)*t^4 + 42*obj.coeff_2(8)*t^5];

        end


        function [err, d_err] = compute_error(obj)
            %METHOD3 Compute joint state error
            err(1)   = obj.q(1) - obj.q_des(1);
            err(2)   = obj.q(2) - obj.q_des(2);
            d_err(1) = obj.dq(1) - obj.q_d_des(1);
            d_err(2) = obj.dq(2) - obj.q_d_des(2);
        end

    end
end

