% Clear workspace
clear; clc; close all;

% Define robot links using DH parameters
% PRR robot: Prismatic - Revolute (rotates vertically) - Revolute (rotates vertically)

L(1) = Link([0 0 0 pi/2, 1]); % Prismatic joint (variable d1)
L(1).qlim = [0, 1];           % Prismatic joint limits [min, max]

L(2) = Link([0 0 0 pi/2]);    % Revolute joint (rotates vertically, about Y-axis)
L(2).qlim = [-pi, pi];        % Revolute joint limits

L(3) = Link([0 0.5 0 pi/2]);  % Revolute joint (rotates vertically, about Y-axis)
L(3).qlim = [-pi/2, pi/2];    % Revolute joint limits

L(4) = Link([0 0.2 0 0, 0]); % Fixed offset of 0.2m from last revolute joint
L(4).qlim = [0, 0]; 
% Create the SerialLink robot
robot = SerialLink(L, 'name', 'PRR Robot');

% Define a desired end-effector pose (transformation matrix)
% Example: Target position at x = 0.4, y = 0.2, z = 0.6
T_target = transl(0, 0, 0.6); % Pose with translation along z-axis only

% Solve inverse kinematics for the desired pose
% Provide an initial guess for joint angles (e.g., [0, 0, 0])
q_guess = [0.5, 0, 0]; % [d1, theta2, theta3]

% Use a mask matrix to specify constraints (only z is considered)
q_ik = robot.ikine(T_target, q_guess, [0 0 1 0 0 0]); % Solve IK

% Display results
disp('Desired end-effector pose:');
disp(T_target);

disp('Calculated joint configuration (q):');
disp(q_ik);

% Visualize the robot in the calculated configuration
robot.plot(q_ik, 'workspace', [-1 1 -1 1 -0.5 1.5]);
title('PRR Robot Inverse Kinematics Solution');
