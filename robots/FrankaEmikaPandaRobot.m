% panda = FrankaEmikaPandaRobot.kinematics() returns a DQ_kinematics object
% using the modified Denavit-Hartenberg parameters of the Franka Emika
% Panda robot

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

classdef FrankaEmikaPandaRobot
    methods (Static)
        function ret = kinematics()
            % Modified D-H of Franka Emika Panda
            mDH_theta = [0, 0, 0, 0, 0, 0, 0];
            mDH_d = [0.333, 0, 3.16e-1, 0, 3.84e-1, 0, 0];
            mDH_a = [0, 0, 0, 8.25e-2, -8.25e-2, 0, 8.8e-2];
            mDH_alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2];
            mDH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,7);
            
            mDH_matrix = [mDH_theta;
                          mDH_d;
                          mDH_a;
                          mDH_alpha;
                          mDH_type]; 
                        
            ret = DQ_SerialManipulatorMDH(mDH_matrix);

            % Set the base's reference frame
            xb = 1 + DQ.E*0.5*DQ([0, 0.0413, 0, 0]);
            ret.set_reference_frame(xb);
            ret.set_base_frame(xb);

            % Set the end-effector
            xe = 1 + DQ.E*0.5*DQ.k*1.07e-1;
            ret.set_effector(xe);
        end

        function ret = dynamics()
            % Modified D-H of Franka Emika Panda
            mDH_theta = [0, 0, 0, 0, 0, 0, 0];
            mDH_d = [0.333, 0, 3.16e-1, 0, 3.84e-1, 0, 0];
            mDH_a = [0, 0, 0, 8.25e-2, -8.25e-2, 0, 8.8e-2];
            mDH_alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2];
            mDH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,7);
            
            mDH_matrix = [mDH_theta;
                          mDH_d;
                          mDH_a;
                          mDH_alpha;
                          mDH_type]; 
            
            % Dynamic properties
            % Position of the center of mass (V-REP)
            pcm = [  0.0046,  0.0756, -0.0338;
                     0.0000,  0.0372,  0.0691;
                     0.0255, -0.0296, -0.0535;
                     0.0526, -0.0528, -0.3605;
                    -0.0854,  0.0077, -0.0878;
                    -0.0434,  0.0179, -0.0174;
                    -0.0001, -0.0009,  0.0805];
            
            % Mass definition (V-REP)
            mass = [1.0000, 1.0000, 2.0000, 2.0000, 7.0000, 4.1268, 0.3079]';

            % Inertia tensor definition (V-REP)
            inertia_tensor(:,:,1) = [  0.0010,  0.0000, -0.0000;
                                       0.0000,  0.0010,  0.0000;
                                      -0.0000,  0.0000,  0.0010];
            inertia_tensor(:,:,2) = [  0.0010,  0.0000, -0.0000;
                                       0.0000,  0.0010, -0.0000;
                                      -0.0000, -0.0000,  0.0010];
            inertia_tensor(:,:,3) = [  0.0043,  0.0008, -0.0010;
                                       0.0008,  0.0041,  0.0011;
                                      -0.0010,  0.0011,  0.0035];
            inertia_tensor(:,:,4) = [  0.0042,  0.0011,  0.0008;
                                       0.0011,  0.0036, -0.0011;
                                       0.0008, -0.0011,  0.0043];
            inertia_tensor(:,:,5) = [  0.0097, -0.0002,  0.0015;
                                      -0.0002,  0.0105,  0.0003;
                                       0.0015,  0.0003,  0.0079];
            inertia_tensor(:,:,6) = [  0.0046, -0.0001,  0.0001;
                                      -0.0001,  0.0042,  0.0002;
                                       0.0001,  0.0002,  0.0045];
            inertia_tensor(:,:,6) = 1.0e-03*[  0.3079,  0.0000,  0.0000;
                                               0.0000,  0.3079, -0.0000;
                                               0.0000, -0.0000,  0.3079];
            
            % Gravity acceleration definition
            gravity_acceleration = 9.81;
            
            %% The compatibility of the class DQ_SerialManipulatorDynamics with modified D-H needs to be checked!
            ret = DQ_SerialManipulatorDynamics(mDH_matrix, 'standard', pcm, mass, inertia_tensor, gravity_acceleration);
            ret.name = 'PandaRobotDyn';
            
            % Set the end-effector
            xe = 1 + DQ.E*0.5*DQ.k*1.07e-1;
            ret.set_effector(xe);
        end
    end
end