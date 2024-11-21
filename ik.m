% Define target position for the end effector
x_target = 0.4;
y_target = 0.2;
z_target = 0.6;

% Define link length of the robot
link_length = 0.5; % Length of each link in meters

% Compute inverse kinematics
q_ik = inverse_kinematics_prr(x_target, y_target, z_target, link_length);

% Display the result
disp('Calculated joint variables [d1, theta2, theta3]:');
disp(q_ik);

% Visualize the robot at the calculated joint configuration
% Define the robot model using Robotics Toolbox
L(1) = Link([0 0 0 pi/2, 1]); % Prismatic joint (variable d1)
L(1).qlim = [0, 1];           % Prismatic joint limits [min, max]

L(2) = Link([0 0 0 pi/2]);    % Revolute joint (rotates vertically, about Y-axis)
L(2).qlim = [-pi, pi];        % Revolute joint limits

L(3) = Link([0 0.5 0 pi/2]);  % Revolute joint (rotates vertically, about Y-axis)
L(3).qlim = [-pi/2, pi/2];    % Revolute joint limits

L(4) = Link([0 0.2 0 0, 0]); % Fixed offset of 0.2m from last revolute joint
L(4).qlim = [0, 0]; 

robot = SerialLink(L, 'name', 'PRR Robot');
robot.plot(q_ik, 'workspace', [-1 1 -1 1 -0.5 1.5]);
title('PRR Robot Inverse Kinematics Solution');
