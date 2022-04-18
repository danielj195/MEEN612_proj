classdef robot_kin
    %ROBOT_KIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        L1 = 0;
        L2 = 0;
    end
    
    methods
        function obj = robot_kin(len_1, len_2)
            %ROBOT_KIN Construct an instance of this class
            %   Initialize robot link lengths
            obj.L1 = len_1;
            obj.L2 = len_2;
        end

        function [X] = forward_kin(obj, q1, q2)
            %METHOD0 Computes forward kinematics
            X(:,1) = obj.L2*cos(q1).*cos(q2);
            X(:,2) = obj.L2*sin(q1).*cos(q2);
            X(:,3) = obj.L2*sin(q2) + obj.L1;
        end

        function [theta_1, theta_2] = inv_kin(obj, X)
            %METHOD1 Computes inverse kinematics given cartesian
            %coordinates
            D = (X(3)-obj.L1)/obj.L2;

            theta_2_sol_1 = atan2(D,sqrt(1-D^2));
            theta_2_sol_2 = atan2(D,-sqrt(1-D^2));
            
            theta_1_sol_1 = atan2(X(2),X(1));
            theta_1_sol_2 = theta_1_sol_1;
            
            if theta_2_sol_1 >=0
                theta_1 = theta_1_sol_1;
                theta_2 = theta_2_sol_1;
            
            else
                theta_1 = theta_1_sol_2;
                theta_2 = theta_2_sol_2;
            end
        end
        
        function J = jacobian(obj, q)
            %METHOD2 Computes robot jacobian for given joint configuration
            J = [-obj.L2*cos(q(2))*sin(q(1))  -obj.L2*cos(q(1))*sin(q(2));
                  obj.L2*cos(q(2))*cos(q(1))  -obj.L2*sin(q(1))*sin(q(2));
                  0                            obj.L2*cos(q(2))          ];
        end

        function dJ = djacobian(obj, q, dq)
            %METHOD2 Computes first time derivative of jacobian for given joint configuration

            j11 = -obj.L2*(-sin(q(2))*dq(2)*sin(q(1)) + cos(q(2))*cos(q(1))*dq(1));
            j12 = -obj.L2*(-sin(q(1))*dq(1)*sin(q(2)) + cos(q(1))*cos(q(2))*dq(2));
            j21 =  obj.L2*(-sin(q(2))*dq(2)*cos(q(1)) - cos(q(2))*sin(q(1))*dq(1));
            j22 = -obj.L2*(cos(q(1))*dq(1)*sin(q(2)) + sin(q(1))*cos(q(2))*dq(2));
            j31 =  0;
            j32 = -obj.L2*sin(q(2))*dq(2);

            dJ = [j11 j12; j21 j22; j31 j32];
        end

        function ddJ = ddjacobian(obj, q, dq, ddq)
            %METHOD3 Computes second time derivative of jacobian for given joint configuration
            j11 = -obj.L2*((-cos(q(2))*dq(2)^2*sin(q(1)) - sin(q(2))*ddq(2)*sin(q(1)) - sin(q(2))*dq(2)*cos(q(1))*dq(1))+...
                (-sin(q(2))*dq(2)*cos(q(1))*dq(1) - cos(q(2))*sin(q(1))*dq(1)^2 + cos(q(2))*cos(q(1))*ddq(1)));

            j12 = -obj.L2*((-cos(q(1))*dq(1)^2*sin(q(2)) - sin(q(1))*ddq(1)*sin(q(2)) - sin(q(1))*dq(1)*cos(q(2))*dq(2))+...
                (-sin(q(1))*dq(1)*cos(q(2))*dq(2) - cos(q(1))*sin(q(2))*dq(2)^2 + cos(q(1))*cos(q(2))*ddq(2)));
            
            j21 = obj.L2*((-cos(q(2))*dq(2)^2*cos(q(1)) - sin(q(2))*ddq(2)*cos(q(1)) + sin(q(2))*dq(2)*sin(q(1))*dq(1))+...
                (sin(q(2))*dq(2)*sin(q(1))*dq(1) - cos(q(2))*cos(q(1))*dq(1)^2 + cos(q(2))*sin(q(1))*ddq(1)));
            
            j22 = -obj.L2*((-sin(q(1))*dq(1)^2*sin(q(2)) + cos(q(1))*ddq(1)*sin(q(2)) + cos(q(1))*dq(1)*cos(q(2))*dq(2))+...
                (cos(q(1))*dq(1)*cos(q(2))*dq(2) - sin(q(1))*sin(q(2))*dq(2)^2 + sin(q(1))*cos(q(2))*ddq(2)));
            
            j31 = 0;
            
            j32 = -obj.L2*(cos(q(2))*dq(2)^2 + sin(q(2))*ddq(2));
            
            ddJ = [j11 j12; j21 j22; j31 j32];
        end
    end
end

