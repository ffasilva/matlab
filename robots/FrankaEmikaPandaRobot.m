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
            pcm = [  0.0036,  0.0000,  0.0021;
                    -0.0034,  0.0035,  0.0287;
                     0.0665, -0.0553, -0.0392;
                     0.0291, -0.0275, -0.2796;
                    -0.0384,  0.0122, -0.0411;
                    -0.0281, -0.0105,  0.0141;
                     0.0102, -0.0042,  0.0616];
            
            % Mass definition (V-REP)
            mass = [4.9707, 0.6469, 3.2286, 3.5879, 1.2259, 1.6666, 0.7355]';

            % Inertia tensor definition (V-REP)
            inertia_tensor(:,:,1) = [  0.7034, -0.0068, -0.0002;
                                      -0.0068,  0.0092, -0.0192;
                                      -0.0002, -0.0192,  0.7067];
            inertia_tensor(:,:,2) = [  0.0085,  0.0103,  0.0040;
                                       0.0103,  0.0265, -0.0008;
                                       0.0040, -0.0008,  0.0281];
            inertia_tensor(:,:,3) = [  0.0182,  0.0055, -0.0044;
                                       0.0055,  0.0565,  0.0082;
                                      -0.0044,  0.0082,  0.0529];
            inertia_tensor(:,:,4) = [  0.0677, -0.0039,  0.0277;
                                      -0.0039,  0.0776,  0.0016;
                                       0.0277,  0.0016,  0.0324];
            inertia_tensor(:,:,5) = [  0.0109,  0.0046, -0.0022;
                                       0.0046,  0.0394, -0.0015;
                                      -0.0022, -0.0015,  0.0315];
            inertia_tensor(:,:,6) = [  0.0025, -0.0001, -0.0015;
                                      -0.0001,  0.0118, -0.0001;
                                      -0.0015, -0.0001,  0.0106];
            inertia_tensor(:,:,7) = [  0.0153, -0.0004, -0.0017;
                                      -0.0004,  0.0129, -0.0005;
                                      -0.0017, -0.0005,  0.0049];
            
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