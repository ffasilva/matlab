% Concrete class that defines the dynamics of free-flying robots.
%
% Usage: robot = DQ_FreeFlyingRobotDynamics(x, position_center_mass, mass, inertia_tensor, gravity_acceleration_modulus)
% - 'x' is a unit dual quaternion of the robot configuration.
% - 'position_center_mass' = [x,y,z] is a 3 x 1 vector of the position of
% the robot's center of mass.
% - 'mass' is the mass of the robot.
% - 'inertia_tensor' is the inertia tensor the robot.
% - 'gravity_acceleration_modulus' is the modulus of the acceleration of
% the gravity.
%
% DQ_HolonomicBaseDynamics Methods (Protected):
%       backward_recursion - Return the wrenche of the center of mass of the robot.
%       forward_recursion - Return the twist, and its time derivative, of the center of mass of the robot.
%       update_gravity_acceleration - Update the gravity acceleration expressed in the center of mass of the robot.
%       update_reference_frames - Update the reference frames 'base_frame', 'reference_frame', and of the center of mass of the robot.
% DQ_HolonomicBaseDynamics Methods (Concrete):
%       get_generalized_forces_from_wrenches - Get the holonomic robot's vector of generalized forces from its wrench.
%       get_joint_configuration - Get the robot configuration vector 'q', 'q_dot', and 'q_dot_dot'.
%       get_number_bodies - Get the number of bodies in the kinematic chain.
%       get_pose_center_mass - Get the pose of the center of mass of the robot.
%       get_pose_end_link - Wrapper function to the DQ_BranchedWholeBody class. Returns a DQ(1).
%       get_twist_ci_rel_to_0_in_ci - Get the twist of the center of mass of the robot.
%       get_twist_ci_rel_to_0_in_ci_dot - Get the time derivative of the twist of the center of mass of the robot.
%       get_wrench_end_effector - Wrapper function of get_wrench_external() to the DQ_BranchedWholeBody class.
%       get_wrench_external - Get the external wrench applied to the robot.
%       set_joint_configuration - Set the vectors 'q', 'q_dot', and 'q_dot_dot' of the robot configuration and calls 'update_reference_frames()'.
%       set_pose_center_mass - Set the pose of the center of mass of the robot.
%       set_position_center_mass - Set the position of the center of mass of the robot and calls 'update_reference_frames()'.
%       set_wrench_end_effector - Wrapper function of set_wrench_external() to the DQ_BranchedWholeBody class.
%       set_wrench_external - Set the external wrench applied to the robot.
%       euler_lagrange - Return the matrices of the canonical Euler-Lagrange model of the robot.
%       newton_euler - Return the wrench on the center of mass of the robot.
% See also DQ_Dynamics.

% (C) Copyright 2011-2023 DQ Robotics Developers
% 
% This file is part of DQ Robotics.
% 
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Frederico Fernandes Afonso Silva - frederico.silva@ieee.org

classdef DQ_FreeFlyingRobotDynamics < DQ_Dynamics & DQ_FreeFlyingRobot
    properties (Access = protected)
        % Pose variables
        position_center_mass;
        pose_center_mass;
        
        % Twists of the robot
        twist_ci_rel_to_0_in_ci;
        twist_ci_rel_to_0_in_ci_dot;
        
        % Wrenches of the robot
        wrench_cm_in_b;
        wrench_ext_source_in_b;
    end
    
    methods (Access = protected)
        function backward_recursion(obj)
            % BACKWARD_RECURSION() calculates the wrench of center of mass
            % of the holonomic robot, expressed on its base reference frame.

            % Updates gravity acceleration of every mass center of the
            % robot
            obj.update_gravity_acceleration();
            
            % Calculation of wrench due to Newton's second law and
            % Euler's rotation equation
            mass = obj.get_mass();
            inertia_tensor = obj.get_inertia_tensor();
            
            f = mass*(obj.twist_ci_rel_to_0_in_ci_dot.D + ...
                        cross(obj.twist_ci_rel_to_0_in_ci.P, obj.twist_ci_rel_to_0_in_ci.D));
            tal = M3(inertia_tensor, obj.twist_ci_rel_to_0_in_ci_dot.P) + ...
                    cross(obj.twist_ci_rel_to_0_in_ci.P, M3(inertia_tensor, obj.twist_ci_rel_to_0_in_ci.P));

            sigma = f + DQ.E*(tal);

            % Check if wrench is a pure dual quaternion as it should
            if(~is_pure(sigma))
                formatSpec = 'Wrench of the base:\n';
                fprintf(formatSpec);
                display(sigma);
                error('Error! Wrench should be a pure dual quaternion!');
            end
            
            % Inclusion of the effects of the gravity acceleration
            wrench_ci_rel_to_0_in_ci = sigma - mass*obj.gravity_acceleration;

            % Check if wrench is a pure dual quaternion as it should
            if(~is_pure(wrench_ci_rel_to_0_in_ci))
                formatSpec = 'Wrench of the base:\n';
                fprintf(formatSpec);
                display(wrench_ci_rel_to_0_in_ci);
                error('Error! Wrench should be a pure dual quaternion!');
            end
            
            % Calculation of the wrench (includes external wrench at the body---e.g., desired body wrench)
            wrench = Ad(obj.pose_center_mass, wrench_ci_rel_to_0_in_ci) + obj.wrench_ext_source_in_b;
            
            % Check if wrench is a pure dual quaternion as it should
            if(~is_pure(wrench))
                disp('Wrench of the body:');
                display(wrench);
                error('Error! Wrench should be a pure dual quaternion!');
            end
            obj.wrench_cm_in_b(1,:) = wrench;
        end
        
        function forward_recursion(obj)
            % FORWARD_RECURSION() calculates the twist of the mass center
            % of the holonomic robot and its time derivative.
            
            % Vectors of joint configuration
            [~, q_dot, q_dot_dot] = obj.get_joint_configuration();
            
            % Calculation of the twist            
            w_0_b_in_0 = q_dot(3)*DQ.k;        
            v_0_b_in_0 = q_dot(1)*DQ.i + q_dot(2)*DQ.j;
            xi_0_b_in_0 = w_0_b_in_0 + DQ.E*v_0_b_in_0;
            
            xi_0_b_in_b = Ad((obj.base_frame.P)', xi_0_b_in_0); % Ad(r_0_in_b)xi_0_b_in_0            
            xi_0_cm_in_cm = Ad((obj.pose_center_mass)', xi_0_b_in_b);
            
            % Check if twist is a pure dual quaternions as it should
            if(~is_pure(xi_0_cm_in_cm))
                disp('Twist of the center of mass:');
                display(xi_0_cm_in_cm);
                error('Error! Twist should be a pure dual quaternion!');
            end
            obj.twist_ci_rel_to_0_in_ci(1,:) = xi_0_cm_in_cm;
            
            % Calculation of the time derivative of the twist
            w_0_b_in_0_dot = q_dot_dot(3)*DQ.k;        
            v_0_b_in_0_dot = q_dot_dot(1)*DQ.i + q_dot_dot(2)*DQ.j;
            xi_0_b_in_0_dot = w_0_b_in_0_dot + DQ.E*v_0_b_in_0_dot;
            
            xi_0_b_in_b_dot = Ad((obj.base_frame.P)', xi_0_b_in_0_dot); % Ad(r_0_in_b)xi_0_b_in_0_dot
            xi_cm_b_in_cm = -xi_0_cm_in_cm + Ad((obj.pose_center_mass)', xi_0_b_in_b); %xi_cm_0_in_cm + xi_0_b_in_cm
            xi_0_cm_in_cm_dot = Ad((obj.pose_center_mass)', xi_0_b_in_b_dot + cross(xi_0_b_in_b.P', xi_0_b_in_b)) + ...
                                  cross(xi_cm_b_in_cm, xi_0_cm_in_cm);
            
            % Check if time derivative of twist is a pure dual quaternions as it should
            if(~is_pure(xi_0_cm_in_cm_dot))
                disp('Time derivative of the twist the center of mass:');
                display(xi_0_cm_in_cm_dot);
                error('Error! Time derivative of the twist should be a pure dual quaternion!');
            end            
            obj.twist_ci_rel_to_0_in_ci_dot(1,:) = xi_0_cm_in_cm_dot;
        end
        
        function update_gravity_acceleration(obj)
            % UPDATE_GRAVITY_ACCELERATION() updates the gravity
            % acceleration expressed in the mass center of the robot.
            gravity_acceleration_modulus = obj.get_gravity_acceleration_modulus();
            gravity_acc_aux = gravity_acceleration_modulus*Ad((obj.base_frame.P)',-DQ.k); % Takes the base rotation in consideration
            r = obj.pose_center_mass.P;
            obj.set_gravity_acceleration(Ad(r',gravity_acc_aux))
        end
        
        function update_reference_frames(obj)
            % UPDATE_REFERENCE_FRAMES() updates the reference frames
            % 'base_frame', 'reference_frame', and 'pose_center_mass'.
            
            % Sets the 'base_frame' and the 'reference_frame' of the robot
            x_base = obj.fkm(obj.q);
            obj.set_base_frame(x_base);
            obj.set_reference_frame(x_base);
            
            % Sets the pose of the center of mass with respect to the
            % 'reference_frame'
            r = DQ(1);
            p = DQ(obj.position_center_mass);
            x_cm = r + (1/2)*DQ.E*p*r;

            obj.pose_center_mass = x_cm;
        end
    end
    
    methods
        function obj = DQ_FreeFlyingRobotDynamics(q, position_center_mass, mass, inertia_tensor, gravity_acceleration_modulus)
            % Kinematic properties inherited from DQ_HolonomicBase
            obj@DQ_HolonomicBase();
            
            % Robot configuration vector definition
            obj.q = q;
            
            % Dynamic properties
            % Position of the center of mass
            obj.set_position_center_mass(position_center_mass);
            
            % Mass definition
            obj.set_mass(mass);

            % Inertia tensor definition
            obj.set_inertia_tensor(inertia_tensor);
            
            % Definition of robot's configuration
            obj.set_joint_configuration(obj.q, zeros(length(obj.q),1), zeros(length(obj.q),1));
            
            % Definition of the modulus of the acceleration of the gravity
            obj.set_gravity_acceleration_modulus(gravity_acceleration_modulus);
            
            % Updates gravity acceleration expressed in the mass center of
            % the robot.
            obj.update_gravity_acceleration();
            
            % Definition of twists and their time derivatives
            obj.twist_ci_rel_to_0_in_ci = DQ();
            obj.twist_ci_rel_to_0_in_ci_dot = DQ();
            
            % Definition of wrenches
            obj.wrench_cm_in_b = DQ();
            obj.wrench_ext_source_in_b = DQ();
        end
        
        function vec_generalized_forces = get_generalized_forces_from_wrenches(obj, wrench)
            % GET_GENERALIZED_FORCES_FROM_WRENCHES(wrench) gets the vector
            % of generalized forces 'vec_generalized_forces' from the
            % 'wrench' considering the 'base_frame'.
            wrench = Ad(obj.get_base_frame.P, wrench);
            
            vec_generalized_forces = zeros(obj.get_dim_configuration_space,1);
            vec_generalized_forces(1) = dot(wrench, DQ.i).P; % generalized force is the force acting in the axis x
            vec_generalized_forces(2) = dot(wrench, DQ.j).P; % generalized force is the force acting in the axis y
            vec_generalized_forces(3) = dot(wrench, DQ.k).D; % generalized force is the torque acting about the axis z
        end
        
        function [q, q_dot, q_dot_dot] = get_joint_configuration(obj)
            % GET_JOINT_CONFIGURATION() gets the configuration vectors 'q',
            % 'q_dot', and 'q_dot_dot'.
            q = obj.q;
            q_dot = obj.get_q_dot();
            q_dot_dot = obj.get_q_dot_dot();
        end
        
        function ret = get_number_bodies(~)
            % GET_NUMBER_BODIES() gets the number of bodies in the
            % kinematic chain.
            ret = 1;
        end
        
        function pose_center_mass = get_pose_center_mass(obj)
            % GET_POSE_CENTER_MASS() gets the pose of the center of mass of
            % the robot.
            pose_center_mass = obj.pose_center_mass;
        end
        
        function pose_end_link = get_pose_end_link(~)
            % GET_POSE_END_LINK() gets the pose of the reference frame of
            % the base. Used as interface wrapper function for the
            % DQ_BranchedWholeBody class.
            pose_end_link = DQ(1);
        end
        
        function twist_ci_rel_to_0_in_ci = get_twist_ci_rel_to_0_in_ci(obj)
            % GET_TWIST_CI_REL_TO_0_IN_CI() gets the twist of the center
            % of mass of the robot.
            twist_ci_rel_to_0_in_ci = obj.twist_ci_rel_to_0_in_ci;
        end
        
        function twist_ci_rel_to_0_in_ci_dot = get_twist_ci_rel_to_0_in_ci_dot(obj)
            % GET_TWIST_CI_REL_TO_0_IN_CI_DOT() gets the time derivative of
            % the twist of the center of mass of the robot.
            twist_ci_rel_to_0_in_ci_dot = obj.twist_ci_rel_to_0_in_ci_dot;
        end
        
        function wrench_external = get_wrench_end_effector(obj)
            % GET_WRENCH_END_EFFECTOR() gets the external wrench applieed
            % to the robot. Used as interface wrapper function for the
            % DQ_BranchedWholeBody class.
            wrench_external = obj.wrench_ext_source_in_b;
        end
        
        function wrench_external = get_wrench_external(obj)
            % GET_WRENCH_EXTERNAL() gets the external wrench applied to the
            % robot.
            wrench_external = obj.wrench_ext_source_in_b;
        end
        
        function set_joint_configuration(obj,q,q_dot,q_dot_dot)
            % SET_JOINT_CONFIGURATION(q,q_dot,q_dot_dot) sets the vectors
            % 'q', 'q_dot' and 'q_dot_dot' of the robot joint configuration
            % and of its first and second order time derivative,
            % respectively, and updates 'pose_end_link',
            % 'pose_center_mass'.
            
            obj.q = q;
            obj.set_q_dot(q_dot);
            obj.set_q_dot_dot(q_dot_dot);
            
            % Updates the reference frame 'pose_center_mass' of the mass
            % center of the robot.
            obj.update_reference_frames();
        end
        
        function set_pose_center_mass(obj, pose_center_mass)
            % SET_POSE_CENTER_MASS(pose_center_mass) sets the pose of the
            % center of mass of the robot.
            obj.pose_center_mass = pose_center_mass;
        end
        
        function set_position_center_mass(obj, position_center_mass)
            % SET_POSITION_CENTER_MASS(position_center_mass) sets the
            % position of the center of mass of the robot.
            
            obj.position_center_mass = position_center_mass;
            
            % Updates the reference frame 'pose_center_mass' of the mass
            % center of the robot.
            obj.update_reference_frames();
        end
        
        function set_wrench_end_effector(obj, wrench_external)
            % SET_WRENCH_END_EFFECTOR(wrench_external) sets the
            % external wrench applied to the robot. Used as interface
            % wrapper function for the DQ_BranchedWholeBody class.
            obj.wrench_ext_source_in_b = wrench_external;
        end
        
        function set_wrench_external(obj, wrench_external)
            % SET_WRENCH_EXTERNAL(wrench_external) sets the external wrench
            % applied to the robot.
            obj.wrench_ext_source_in_b = wrench_external;
        end
        
        function [M, c, g] = euler_lagrange(obj)
            % EULER_LAGRANGE() Returns the matrice 'M' and the vectors 'c'
            % and 'g' of the canonical Euler-Lagrange model of the robot.
            % That model is obtained through the Newton-Euler formalism.
            
            % Storage of joint configuration, gravity acceleration modulus
            % and external wrench to return them to their previous
            % values after the obtention of the model.
            [q, q_dot, q_dot_dot] = obj.get_joint_configuration();
            gravity_acceleration_modulus = obj.get_gravity_acceleration_modulus();
            wrench_external = obj.get_wrench_external();

            % Obtention of g(q)
            dim_configuration_space = obj.get_dim_configuration_space;
            q_zeros = zeros(dim_configuration_space,1);
            
            obj.set_wrench_external(DQ(0)); % Removes external wrench (modelling does not considers it)  
            obj.set_joint_configuration(q, q_zeros, q_zeros);
            
            g = obj.get_generalized_forces_from_wrenches(obj.newton_euler());

            % Obtention of c(q,q_dot)
            obj.set_gravity_acceleration_modulus(0); % Gravity must be set to zero for the calculation of 'c(q,q_dot)' and 'M(q)'.
            obj.set_joint_configuration(q, q_dot, q_zeros);
            
            c = obj.get_generalized_forces_from_wrenches(obj.newton_euler());

            % Obtention of M(q)
            I = eye(length(obj.q),length(obj.q));
            M = zeros(dim_configuration_space, dim_configuration_space);
            for i=1:length(obj.q)
                obj.set_joint_configuration(q, q_zeros, I(:,i));

                M(:,i) = obj.get_generalized_forces_from_wrenches(obj.newton_euler());
            end

            % Check if matrix M(q) is symmetric and positive definite
            if(~issymmetric(round(M,6)))
                error('Something is wrong! M(q) is not symmetric!')
            end

            [~,pd] = chol(M);
            if(pd ~= 0)
                error('Something is wrong! M(q) is not positive definite!')
            end
            
            % Returns joint configuration, gravity acceleration modulus and
            % external wrench to their previous values.
            obj.set_joint_configuration(q, q_dot, q_dot_dot);
            obj.set_gravity_acceleration_modulus(gravity_acceleration_modulus);
            obj.set_wrench_external(wrench_external);
        end
        
        function wrench_cm_in_b = newton_euler(obj)
            % NEWTON_EULER() Returns the wrench 'wrench_cm_in_b' of the
            % center of mass of the robot expressed on its own reference
            % frame.
        
            % Forward calculation (twists and their time derivatives)
            obj.forward_recursion();

            % Backward calculation (wrenches)
            obj.backward_recursion();
            
            wrench_cm_in_b = obj.wrench_cm_in_b;
        end
    end
end