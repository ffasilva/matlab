% Create a new KUKA LW4 robot manipulator
% KukaLwr4Robot Methods (Static):
%   kinematics - Return a DQ_SerialManipulator object with the KUKA LW4 robot manipulator kinematic parameters

% (C) Copyright 2011-2019 DQ Robotics Developers
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
%     Bruno Vihena Adorno - adorno@ufmg.br

classdef KukaLwr4Robot
    methods (Static)
        function ret = kinematics()
            %Standard D-H of KUKA-LWR
            kuka_DH_theta=[0, 0, 0, 0, 0, 0, 0];
            kuka_DH_d = [0.310, 0, 0.4, 0, 0.39, 0, 0];
            kuka_DH_a = [0, 0, 0, 0, 0, 0, 0];
            kuka_DH_alpha = [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0];
            kuka_DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,7);
            kuka_DH_matrix = [kuka_DH_theta;
                kuka_DH_d;
                kuka_DH_a;
                kuka_DH_alpha;
                kuka_DH_type];

            ret = DQ_SerialManipulatorDH(kuka_DH_matrix,'standard');
        end

        function ret = dynamics()
             %Standard D-H of KUKA-LWR
            kuka_DH_theta=[0, pi, 0, 0, 0, 0, 0];
            kuka_DH_d = [0.310, 0, 0.4, 0, 0.39, 0, 0];
            kuka_DH_a = [0, 0, 0, 0, 0, 0, 0];
            kuka_DH_alpha = [pi/2, pi/2, -pi/2, pi/2, pi/2, -pi/2, 0];
            kuka_DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,7);
            kuka_DH_matrix = [kuka_DH_theta;
                kuka_DH_d;
                kuka_DH_a;
                kuka_DH_alpha;
                kuka_DH_type];
            
            % Dynamic properties
            % Position of the center of mass (V-REP)
            pcm = [  0.0002, -0.0804,  0.0257;
                     0.0000,  0.0277, -0.1112;
                    -0.0001, -0.0804, -0.0277;
                     0.0000, -0.0257, -0.1111;
                     0.0000, -0.0956,  0.0254;
                     0.0002,  0.0088, -0.0733;
                     0.0000, -0.0005,  0.0000];
            
            % Mass definition (V-REP)
            mass = [2.7000, 2.7000, 2.7000, 2.7000, 1.7000, 1.6000, 0.3000]';

            % Inertia tensor definition (V-REP)
            inertia_tensor(:,:,1) = [  0.0163, -0.0000, -0.0000;
                                      -0.0000,  0.0050, -0.0035;
                                      -0.0000, -0.0035,  0.0162];
            inertia_tensor(:,:,2) = [  0.0163, -0.0000,  0.0000;
                                      -0.0000,  0.0162,  0.0035;
                                       0.0000,  0.0035,  0.0050];
            inertia_tensor(:,:,3) = [  0.0163,  0.0000, -0.0000;
                                       0.0000,  0.0050,  0.0035;
                                      -0.0000,  0.0035,  0.0162];
            inertia_tensor(:,:,4) = [  0.0163, -0.0000, -0.0000;
                                      -0.0000,  0.0162, -0.0035;
                                      -0.0000, -0.0035,  0.0050];
            inertia_tensor(:,:,5) = [  0.0098,  0.0000,  0.0000;
                                       0.0000,  0.0037, -0.0031;
                                       0.0000, -0.0031,  0.0091];
            inertia_tensor(:,:,6) = [  0.0030,  0.0000, -0.0000;
                                       0.0000,  0.0030, -0.0000;
                                      -0.0000, -0.0000,  0.0034];
            inertia_tensor(:,:,7) = 1.0e-03*[  0.1017,  0.0000, -0.0000;
                                               0.0000,  0.1017, -0.0000;
                                              -0.0000, -0.0000,  0.1584];
            
            % Gravity acceleration definition
            gravity_acceleration = 9.81;
            
            ret = DQ_SerialManipulatorDynamics(kuka_DH_matrix, 'standard', pcm, mass, inertia_tensor, gravity_acceleration);
            ret.name = 'KUKALBR4Dyn';

            % Set the end-effector
            ret.set_effector(1+0.5*DQ.E*DQ.k*0.07);
        end

    end
end
