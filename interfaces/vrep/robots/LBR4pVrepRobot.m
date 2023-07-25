% CLASS LBR4pVrepRobot - Concrete class to interface with the "KUKA LBR4+"
% robot in VREP.
%
% Usage:
%       1) Drag-and-drop a "KUKA LBR4+" robot to a VREP scene.
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
%   LBR4pVrepRobot Methods:
%       send_q_to_vrep - Sends the joint configurations to VREP
%       get_q_from_vrep - Obtains the joint configurations from VREP
%       kinematics - Obtains the DQ_Kinematics implementation of this robot

% (C) Copyright 2020 DQ Robotics Developers
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
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%     Murilo Marques Marinho - murilo@nml.t.u-tokyo.ac.jp

classdef LBR4pVrepRobot < DQ_VrepRobot
    
    properties
        joint_names;
        base_frame_name;
        link_names = {};
        lua_script_name;
    end

    methods (Access = protected)
        function update_dynamic_parameters(obj, robot_dynamics)
            n = robot_dynamics.dim_configuration_space;
            mass = zeros(n,1);
            center_of_mass = zeros(n,3);
            inertia_tensor = zeros(3,3,n);
            for i=1:n
                mass(i) = obj.vrep_interface.get_mass(obj.link_names{i});
                if(i<n)
                    center_of_mass(i,:) = obj.vrep_interface.get_center_of_mass(obj.link_names{i}, obj.joint_names{i+1});
                    inertia_tensor(:,:,i) = obj.vrep_interface.get_inertia_matrix(obj.link_names{i}, obj.joint_names{i+1});
                else
                    center_of_mass(i,:) = obj.vrep_interface.get_center_of_mass(obj.link_names{i}, obj.joint_names{i});
                    inertia_tensor(:,:,i) = obj.vrep_interface.get_inertia_matrix(obj.link_names{i}, obj.joint_names{i});
                end
            end

            robot_dynamics.set_mass(mass);
            robot_dynamics.set_position_center_mass(center_of_mass);
            robot_dynamics.set_inertia_tensor(inertia_tensor);
        end
    end
    
    methods
        function obj = LBR4pVrepRobot(robot_name,vrep_interface)
            %% Constructs an instance of a LBR4pVrepRobot
            %  >> vi = VrepInterface()
            %  >> vi.connect('127.0.0.1',19997);
            %  >> robot = LBR4pVrepRobot("LBR4p", vi)
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            % From the second copy of the robot and onward, VREP appends a
            % #number in the robot's name. We check here if the robot is
            % called by the correct name and assign an index that will be
            % used to correctly infer the robot's joint labels.
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'LBR4p')
                error('Expected LBR4p')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            % Initialize joint names, link names, and base frame name
            obj.joint_names = {};
            for i=1:7
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');

                current_link_name = {robot_label,'_link',int2str(i+1),robot_index};
                obj.link_names{i} = strjoin(current_link_name,'');
            end
            obj.base_frame_name = obj.joint_names{1};
        end
        
        function send_q_to_vrep(obj,q)
            %% Sends the joint configurations to VREP
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi)
            %  >> q = zeros(7,1);
            %  >> vrep_robot.send_q_to_vrep(q)
            obj.vrep_interface.set_joint_positions(obj.joint_names,q)
        end
        
        function q = get_q_from_vrep(obj)
            %% Obtains the joint configurations from VREP
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi)
            %  >> q = vrep_robot.get_q_from_vrep(q)
            q = obj.vrep_interface.get_joint_positions(obj.joint_names);
        end

        function send_q_dot_to_vrep(obj,q_dot)
            %% Sends the joint velocities to VREP
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi)
            %  >> q_dot = zeros(7,1);
            %  >> vrep_robot.send_q_dot_to_vrep(q_dot)
            obj.vrep_interface.set_joint_target_velocities(obj.joint_names,q_dot);
        end
        
        function qd = get_q_dot_from_vrep(obj)
            %% Obtains the joint configurations from VREP
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi)
            %  >> qd = vrep_robot.get_q_dot_from_vrep()
            qd = obj.vrep_interface.get_joint_velocities(obj.joint_names);
        end
        
        function send_tau_to_vrep(obj,tau)
            %% Sends the joint torques to VREP
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi)
            %  >> tau = zeros(7,1);
            %  >> vrep_robot.send_tau_to_vrep(tau)
            obj.vrep_interface.set_joint_torques(obj.joint_names,tau)
        end
        
        function tau = get_tau_from_vrep(obj)
            %% Obtains the joint torques from VREP
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi)
            %  >> tau = vrep_robot.get_tau_from_vrep()
            tau = obj.vrep_interface.get_joint_torques(obj.joint_names);
        end
        
        function kin = kinematics(obj)
            %% Obtains the DQ_SerialManipulator instance that represents this LBR4p robot.
            %  >> vrep_robot = LBR4pVrepRobot("LBR4p", vi)
            %  >> robot_kinematics = vrep_robot.kinematics()
            
            LBR4p_DH_theta=[0, 0, 0, 0, 0, 0, 0];
            LBR4p_DH_d = [0.200, 0, 0.4, 0, 0.39, 0, 0];
            LBR4p_DH_a = [0, 0, 0, 0, 0, 0, 0];
            LBR4p_DH_alpha = [pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0];
            LBR4p_DH_type = double(repmat(DQ_JointType.REVOLUTE,1,7));
            LBR4p_DH_matrix = [LBR4p_DH_theta;
                LBR4p_DH_d;
                LBR4p_DH_a;
                LBR4p_DH_alpha
                LBR4p_DH_type];
            
            kin = DQ_SerialManipulatorDH(LBR4p_DH_matrix);
            
            kin.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            kin.set_effector(1+0.5*DQ.E*DQ.k*0.07);
        end

        function dyn = dynamics(obj, lua_script_name)
            %% Obtains the DQ_SerialManipulatorDynamics instance that represents this Jaco robot.
            %  >> vrep_robot = JacoVrepRobot('Jaco', vi)
            %  >> lua_script_name = 'MyLuaScript'
            %  >> robot_dynamics = vrep_robot.dynamics(lua_script_name)
            
            % Set the Lua script name
            obj.lua_script_name = lua_script_name;
            
            % Create a DQ_SerialManipulatorDynamics object
            dyn = KukaLwr4Robot.dynamics;

            % Set the robot configuration with V-REP values
            q_read = obj.get_q_from_vrep(); 
            q_read_dot = obj.get_q_dot_from_vrep();
            q_read_dot_dot = zeros(dyn.get_dim_configuration_space,1);

            dyn.set_joint_configuration(q_read, q_read_dot, q_read_dot_dot);
            
            % Update base and reference frame with V-REP values
            dyn.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            dyn.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            
            % Update dynamic parameters with V-REP information
            obj.update_dynamic_parameters(dyn);
        end
        
    end
end

