% CLASS DQ_BranchedVrepRobot - Concrete class to interface with branched
% robots in VREP.
%
% Usage:
%       1) Drag-and-drop a branched robot to a VREP scene. For instance, a
%       "BM_TRO2023" robot.
%       2) Run
%           >> vi = DQ_VrepInterface();
%           >> vi.connect('127.0.0.1',19997);
%           >> vrep_robot = LBR4pVrepRobot("BM_TRO2023", vi);
%           >> vi.start_simulation();
%           >> robot.get_q_from_vrep();
%           >> pause(1);
%           >> vi.stop_simulation();
%           >> vi.disconnect();
%       Note that the name of the robot should be EXACTLY the same as in
%       VREP. For instance, if you drag-and-drop a second robot, its name
%       will become "BM_TRO2023#0", a third robot, "BM_TRO2023#1", and so
%       on.
%
%   DQ_BranchedVrepRobot Methods (Protected):
%       update_dynamic_parameters - Updates the dynamic parameters of the branched robot in the CoppeliaSim scene
%       update_branch_dynamic_parameters - Updates the dynamic parameters of a branch from the branched robot in the CoppeliaSim scene

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

classdef DQ_BranchedVrepRobot < DQ_SerialVrepRobot
    methods (Access = protected)
        function update_dynamic_parameters(obj, robot_dynamics)
            % This method updates the dynamic parameters of the branched robot in the CoppeliaSim scene.
            % Usage:
            %     update_dynamic_parameters(robot_dynamics);
            %          robot_dynamics: A DQ_BranchedWholeBody object representing the robot in the CoppeliaSim scene.
            %     
            %     Note that this is a protected method and, as such, can only be used by classes that inherit from
            %     DQ_BranchedVrepRobot.

            name_index = 1;
            for i=1:length(robot_dynamics.get_chain)
                formatSpec = 'Updating dynamic parameters of the %ith branch...\n';
                fprintf(formatSpec,i);
                obj.update_branch_dynamic_parameters(robot_dynamics.get_chain{i}, name_index);

                name_index = name_index + length(robot_dynamics.get_chain{i}.q);
            end
        end

        function update_branch_dynamic_parameters(obj, robot_dynamics, name_index)
            % This method updates the dynamic parameters of a branch from the branched robot in the CoppeliaSim scene.
            % Usage:
            %     update_branch_dynamic_parameters(obj, robot_dynamics, name_index)
            %          robot_dynamics: A DQ_SerialManipulatorDynamics object representing a branch from the robot in the CoppeliaSim scene.
            %          name_index: Index indicating in which range from the properties 'joint_names'/'link_names' the link/joint names of this branch are stored.
            %     
            %     Note that this is a protected method and, as such, can only be used by classes that inherit from
            %     DQ_BranchedVrepRobot.

            q_read = robot_dynamics.get_joint_configuration;
            n = robot_dynamics.dim_configuration_space;
            mass = zeros(n,1);
            center_of_mass = zeros(n,3);
            inertia_tensor = zeros(3,3,n);
            for i=1:n
                % Get the link's mass
                mass(i) = obj.vrep_interface.get_mass(obj.link_names{name_index});

                % Get the link's center of mass position with respect to the end-link frame
                pcm_in_0 = DQ(obj.vrep_interface.get_center_of_mass(obj.link_names{name_index}, -1));
                rcm_in_0 = obj.vrep_interface.get_object_rotation(obj.link_names{name_index});
                xcm_in_0 = rcm_in_0 + (1/2)*DQ.E*pcm_in_0*rcm_in_0;
                xl_in_0 = robot_dynamics.fkm(q_read,i);
                xcm_in_xl = (xl_in_0')*xcm_in_0;
                center_of_mass(i,:) = vec3(xcm_in_xl.translation)';
                
                % Get the link's inertia tensor with respect to the end-link frame
                inertia_tensor_in_0 = obj.vrep_interface.get_inertia_matrix(obj.link_names{name_index}, -1);
                Rcm_in_0 = rotation_matrix_from_quaternion(xcm_in_0.P);
                inertia_tensor(:,:,i) = Rcm_in_0'*inertia_tensor_in_0*Rcm_in_0;

                name_index = name_index + 1;
            end

            robot_dynamics.set_mass(mass);
            robot_dynamics.set_position_center_mass(center_of_mass);
            robot_dynamics.set_inertia_tensor(inertia_tensor);
        end
    end

    methods
        function obj = DQ_BranchedVrepRobot(base_robot_name, robot_dof, robot_name, vrep_interface)
            % This method constructs an instance of a DQ_BranchedVrepRobot.
            % Usage:
            %     DQ_BranchedVrepRobot(base_robot_name, robot_dof, robot_name, vrep_interface);  
            %          base_robot_name: The base name of the robot in the CoppeliaSim scene.
            %          robot_dof: The number of DoF of the robot in the CoppeliaSim scene.
            %          robot_name: The name of the robot in the CoppeliaSim scene.
            %          vrep_interface: The DQ_VrepInterface object connected to the CoppeliaSim scene.
            %
            % Example: 
            %     vi = DQ_VrepInterface();
            %     vi.connect('127.0.0.1',19997);
            %     vrep_robot = DQ_BranchedVrepRobot("my_robot", 7, "my_robot#1", vi);
            %     
            %     Note that the name of the robot should be EXACTLY the same as in the CoppeliaSim
            %     scene. For instance, if you drag-and-drop a second robot, its name will become 
            %     "my_robot#0", a third robot, "my_robot#1", and so on.

            obj@DQ_SerialVrepRobot(base_robot_name, robot_dof, robot_name, vrep_interface);
        end
    end
end