clc;
clear all;

% Create an instance of NumGenerator
numGen = NumGenerator();

% Configuration parameters
scaling = 50; % Scaling factor
num_points_per_segment = 10; % Number of interpolated points per segment
offset1 = [-7, 0, 0]; % Offset for the starting position of digit 1
offset2 = [-2, 0, 0]; % Offset for the starting position of digit 2
offset3 = [3, 0, 0]; % Offset for the starting position of digit 3
offset4 = [7, 0, 0]; % Offset for the starting position of digit 4

% Get the current time
current_time = datetime('now'); % Fetch the current time
hours = hour(current_time); % Get the hour component
minutes = minute(current_time); % Get the minute component

% Convert the time to digits
time_digits = [floor(hours / 10), mod(hours, 10), floor(minutes / 10), mod(minutes, 10)];

% Dynamically generate the path points for each digit
points1 = numGen.generate_coord(time_digits(1), num_points_per_segment, scaling, offset1);
points2 = numGen.generate_coord(time_digits(2), num_points_per_segment, scaling, offset2);
points3 = numGen.generate_coord(time_digits(3), num_points_per_segment, scaling, offset3);
points4 = numGen.generate_coord(time_digits(4), num_points_per_segment, scaling, offset4);

% Combine path points into a list
points_list = {points1, points2, points3, points4};

% Plot paths for verification
for i = 1:length(points_list)
    myplot1(points_list{i});
end

% Define the PRR robot
L(1) = Link([0 0 0 0, 1]); % Prismatic joint (variable d1)
L(1).qlim = [0, 20]; % Prismatic joint limits [min, max]

L(2) = Link([0 0 10 0]); % Revolute joint (rotates about the Y-axis)
L(2).qlim = [-pi, pi]; % Revolute joint limits

L(3) = Link([0 0 10.93 0]); % Revolute joint (rotates about the Y-axis)
L(3).qlim = [-pi, pi]; % Revolute joint limits

L(4) = Link([0 0 0 pi, 0]); % Fixed offset from the last revolute joint
L(4).qlim = [0, 0];

% Create the robot
robot = SerialLink(L, 'name', 'PRR Robot');
robot.teach();

% Initialize inverse kinematics and trajectory variables
q_list = [];
q0 = [0; 0; 0; 0]; % Initial joint angles
z_lift = 1; % Height to lift the Z-axis
pause_time = 0.05; % Pause time for animation steps

% Move from the initial point to the start of the first digit
start_point = [0, 0, 0]; % Assume initial point is the origin
first_start = points_list{1}(1, :); % Starting point of the first digit
interp_steps = 20; % Number of interpolation steps
R = eul2rotm([0, 0, -pi]); % Rotation matrix

for k = 1:interp_steps
    % Interpolate to generate intermediate points
    interp_point = start_point + (first_start - start_point) * (k / interp_steps);
    T = transl(interp_point);
    T(1:3, 1:3) = R;
    q = robot.ikunc(T, q0); % Solve inverse kinematics
    q(1) = max(q(1), 0); % Ensure non-negative prismatic joint
    robot.animate(q); % Update animation
    pause(pause_time);
end

% Update q0 to the joint state at the first digit's starting point
q0 = q;

% Traverse each digit's path
for i = 1:length(points_list)
    points = points_list{i};
    
    % Traverse the current digit's path points
    for j = 1:size(points, 1)
        T = transl(points(j, :)); % Compute transformation matrix
        T(1:3, 1:3) = R;
        q = robot.ikunc(T, q0); % Solve inverse kinematics
        q(1) = max(q(1), 0); % Ensure non-negative prismatic joint
        robot.animate(q); % Update animation
        pause(pause_time);
        q0 = q; % Update the current joint angles
    end
    
    % Lift the Z-axis
    if i < length(points_list) % If not the last digit
        q0(1) = q0(1) + z_lift; % Update Z-axis position
        q0(1) = max(q0(1), 0); % Ensure non-negative Z-axis
        robot.animate(q0); % Animate lifting action
        pause(pause_time);

        % Move to the starting point of the next digit (interpolation transition)
        next_start = points_list{i + 1}(1, :); % Starting point of the next digit
        for k = 1:interp_steps
            interp_point = points(end, :) + (next_start - points(end, :)) * (k / interp_steps);
            T = transl(interp_point);
            T(1:3, 1:3) = R;
            q = robot.ikunc(T, q0);
            q(1) = max(q(1), 0); % Ensure non-negative Z-axis
            robot.animate(q);
            pause(pause_time);
        end

        % Lower the Z-axis at the new starting point
        T_next_start = transl(next_start);
        T_next_start(1:3, 1:3) = R;
        q = robot.ikunc(T_next_start, q0);
        q(1) = q(1) - z_lift; % Lower Z-axis
        q(1) = max(q(1), 0); % Ensure non-negative Z-axis
        robot.animate(q); % Animate lowering action
        pause(pause_time);
        q0 = q; % Update joint state
    end
end
