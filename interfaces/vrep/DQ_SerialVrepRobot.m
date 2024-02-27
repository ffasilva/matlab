% CLASS DQ_SerialVrepRobot - Concrete class to interface with serial robots
% in VREP.
%
% Usage:
%       1) Drag-and-drop a serial robot to a VREP scene. For instance, a
%       "LBR4p" robot.
%       2) Run
%           >> vi = DQ_VrepInterface();
%           >> vi.connect('127.0.0.1',19997);
%           >> vrep_robot = LBR4pVrepRobot("LBR4p", vi);
%           >> vi.start_simulation();
%           >> robot.get_q_from_vrep();
%           >> pause(1);
%           >> vi.stop_simulation();
%           >> vi.disconnect();
%       Note that the name of the robot should be EXACTLY the same as in
%       VREP. For instance, if you drag-and-drop a second robot, its name
%       will become "LBR4p#0", a third robot, "LBR4p#1", and so on.
%
%   DQ_SerialVrepRobot Methods:
%       send_q_to_vrep - Sends the joint configurations to VREP
%       get_q_from_vrep - Obtains the joint configurations from VREP
%       kinematics - Obtains the DQ_Kinematics implementation of this robot
%   DQ_SerialVrepRobot Methods (Protected):
%       update_manipulator_dynamic_parameters - Updates the dynamic parameters of the serial robot in the CoppeliaSim scene

% (C) Copyright 2018-2023 DQ Robotics Developers
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
%     1. Frederico Fernandes Afonso Silva (frederico.silva@ieee.org)
%        - Responsible for the original implementation.

classdef DQ_SerialVrepRobot < DQ_VrepRobot
    properties
        joint_names;
        base_frame_name;
        link_names = {};
        lua_script_name;
    end

    methods (Access = protected)
        function update_dynamic_parameters(obj, robot_dynamics)
            % This method updates the dynamic parameters of the serial robot in the CoppeliaSim scene.
            % Usage:
            %     update_dynamic_parameters(robot_dynamics);
            %          robot_dynamics: A DQ_SerialManipulatorDynamics object representing the robot in the CoppeliaSim scene.
            %     
            %     Note that this is a protected method and, as such, can only be used by classes that inherit from
            %     DQ_SerialVrepRobot.

            q_read = robot_dynamics.get_joint_configuration;
            n = robot_dynamics.dim_configuration_space;
            mass = zeros(n,1);
            center_of_mass = zeros(n,3);
            inertia_tensor = zeros(3,3,n);
            for i=1:n
                % Get the link's mass
                mass(i) = obj.vrep_interface.get_mass(obj.link_names{i});

                % Get the link's center of mass position with respect to the end-link frame
                pcm_in_0 = DQ(obj.vrep_interface.get_center_of_mass(obj.link_names{i}, -1));
                rcm_in_0 = obj.vrep_interface.get_object_rotation(obj.link_names{i});
                xcm_in_0 = rcm_in_0 + (1/2)*DQ.E*pcm_in_0*rcm_in_0;
                xl_in_0 = robot_dynamics.fkm(q_read,i);
                xcm_in_xl = (xl_in_0')*xcm_in_0;
                center_of_mass(i,:) = vec3(xcm_in_xl.translation)';

                % Get the link's inertia tensor with respect to the end-link frame
                inertia_tensor_in_0 = obj.vrep_interface.get_inertia_matrix(obj.link_names{i}, -1);
                Rcm_in_0 = rotation_matrix_from_quaternion(xcm_in_0.P);
                inertia_tensor(:,:,i) = Rcm_in_0'*inertia_tensor_in_0*Rcm_in_0;
            end

            robot_dynamics.set_mass(mass);
            robot_dynamics.set_position_center_mass(center_of_mass);
            robot_dynamics.set_inertia_tensor(inertia_tensor);
        end
    end

    methods
        function obj = DQ_SerialVrepRobot(base_robot_name, robot_dof, robot_name, vrep_interface)
            % This method constructs an instance of a DQ_SerialVrepRobot.
            % Usage:
            %     DQ_SerialVrepRobot(base_robot_name, robot_dof, robot_name, vrep_interface);  
            %          base_robot_name: The base name of the robot in the CoppeliaSim scene.
            %          robot_dof: The number of DoF of the robot in the CoppeliaSim scene.
            %          robot_name: The name of the robot in the CoppeliaSim scene.
            %          vrep_interface: The DQ_VrepInterface object connected to the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     
            %     Note that the name of the robot should be EXACTLY the same as in the CoppeliaSim
            %     scene. For instance, if you drag-and-drop a second robot, its name will become 
            %     "my_robot#0", a third robot, "my_robot#1", and so on.

            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;

            %% The use of 'initialize_joint_names_from_vrep()', as is done in the C++ implementation, is not supported on a constructor in MATLAB
            % From the second copy of the robot and onward, VREP appends a
            % #number in the robot's name. We check here if the robot is
            % called by the correct name and assign an index that will be
            % used to correctly infer the robot's joint labels.
            splited_name = strsplit(obj.robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label, base_robot_name)
                error('Expected %s', base_robot_name)
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            % Initialize joint names, link names, and base frame name
            obj.joint_names = {};
            for i=1:robot_dof
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');

                current_link_name = {robot_label,'_link',int2str(i+1),robot_index};
                obj.link_names{i} = strjoin(current_link_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end

        function joint_names = get_joint_names(obj)
            % This method gets the joint names of the robot in the CoppeliaSim scene.
            % Usage:
            %     get_joint_names;
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     joint_names = vrep_robot.get_joint_names;

            joint_names = obj.joint_names;
        end

        function set_configuration_space_positions(obj, q)
            % This method sets the joint configurations to the robot in the CoppeliaSim scene.
            % Usage:
            %     set_configuration_space_positions(q);  
            %          q: The joint configurations of the robot in the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     q = zeros(7,1);
            %     vrep_robot.set_configuration_space_positions(q);
            %     
            %     Note that this calls "set_joint_positions" in the remoteAPI, meaning that it
            %     is only suitable for passive joints.

            obj.vrep_interface.set_joint_positions(obj.joint_names, q)
        end
        
        function q = get_configuration_space_positions(obj)
            % This method gets the joint configurations of the robot in the CoppeliaSim scene.
            % Usage:
            %     get_configuration_space_positions;  
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     q = vrep_robot.get_configuration_space_positions;

            q = obj.vrep_interface.get_joint_positions(obj.joint_names);
        end

        function set_target_configuration_space_positions(obj, q_target)
            % This method sets the joint configurations to the robot in the CoppeliaSim scene as a target configuration for the joint controllers.
            % Usage:
            %     set_target_configuration_space_positions(q);  
            %          q_target: The target joint configurations of the robot in the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     q_target = zeros(7,1);
            %     vrep_robot.set_target_configuration_space_positions(q_target);
            %     
            %     Note that this calls "set_joint_target_positions" in the remoteAPI, meaning that it
            %     is only suitable for active joints.

            obj.vrep_interface.set_joint_target_positions(obj.joint_names, q_target)
        end
        
        function qd = get_configuration_space_velocities(obj)
            % This method gets the joint velocities of the robot in the CoppeliaSim scene.
            % Usage:
            %     get_configuration_space_velocities;  
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     qd = vrep_robot.get_configuration_space_velocities;
            
            qd = obj.vrep_interface.get_joint_velocities(obj.joint_names);
        end

        function set_target_configuration_space_velocities(obj, v_target)
            % This method sets the joint velocities of the robot in the CoppeliaSim scene as a target velocity for the joint controllers.
            % Usage:
            %     set_target_configuration_space_positions(q);  
            %          v_target: The target joint velocities of the robot in the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     v_target = zeros(7,1);
            %     vrep_robot.set_target_configuration_space_velocities(v_target);
            %     
            %     Note that this calls "set_joint_target_velocities" in the remoteAPI, meaning that it
            %     is only suitable for active joints.

            obj.vrep_interface.set_joint_target_velocities(obj.joint_names, v_target);
        end

        function set_configuration_space_torques(obj,tau)
            % This method sets the joint torques of the robot in the CoppeliaSim scene.
            % Usage:
            %     set_target_configuration_space_positions(q);  
            %          tau: The joint torques of the robot in the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     tau = zeros(7,1);
            %     vrep_robot.set_configuration_space_torques(tau);
            %     
            %     Note that this calls "set_joint_torques" in the remoteAPI, meaning that it
            %     is only suitable for active joints.

            obj.vrep_interface.set_joint_torques(obj.joint_names,tau)
        end

        function tau = get_configuration_space_torques(obj)
            % This method gets the joint torques of the robot in the CoppeliaSim scene.
            % Usage:
            %     get_configuration_space_torques;  
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_SerialVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     tau = vrep_robot.get_configuration_space_torques;

            tau = obj.vrep_interface.get_joint_torques(obj.joint_names);
        end
    end
end