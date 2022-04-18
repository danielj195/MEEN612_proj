classdef Loxodrome < trajectory
    %LOXODROME Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m;
    end
    
    methods
        function obj = Loxodrome(time_vec, L1, L2)
            %LOXODROME Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@trajectory(time_vec, L1, L2);

            syms t x(t) y(t) z(t);
            %Trajectory and derivatives
            obj.m = cot(1);
            x(t) = obj.L2*cos(t)/cosh(obj.m*t);  
            y(t) = obj.L2*sin(t)/cosh(obj.m*t);
            z(t) = obj.L1 + obj.L2*tanh(obj.m*t);
            obj.time = t;
            obj.x = x(t);
            obj.y = y(t);
            obj.z = z(t);
        end
        
    end
end

