% Define the DH parameters for each joint
% a: link length
% alpha: link twist
% d: link offset (assumed zero for simplicity)
% theta: joint angle (variable)

syms theta1 theta2 theta3 alpha1 alpha2 alpha3 % symbolic variables for joint angles and link lengths

a1 = 1; % Length of link 1
a2 = 0; % Length of link 2
a3 = 0; % Length of link 3
d1 = 0; % Offset along z for link 1 (0 for a typical revolute joint)
d2 = 1; % Offset along z for link 2
d3 = 1; % Offset along z for link 3

% Define DH parameter table
DH_params = [theta1, d1, a1, alpha1; % Joint 1
             theta2, d2, a2, alpha2; % Joint 2
             theta3, d3, a3, alpha3]; % Joint 3

% Function to calculate individual transformation matrix based on DH parameters
function T = dh_transform(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
end

% Calculate the transformation matrices for each joint
T01 = dh_transform(DH_params(1,1), DH_params(1,2), DH_params(1,3), DH_params(1,4));
T12 = dh_transform(DH_params(2,1), DH_params(2,2), DH_params(2,3), DH_params(2,4));
T23 = dh_transform(DH_params(3,1), DH_params(3,2), DH_params(3,3), DH_params(3,4));

% Composite transformation from base to end effector
T03 = simplify(T01 * T12 * T23);

% Display the transformation matrix T03
disp('Transformation matrix from base to end-effector (T03):');
disp(T03);

%% Create the manipulator

n = 3; %degrees of freedom

L(1) = Link([0 a1 d1 pi/2]); % Link 1
L(2) = Link([pi/2 a2 d2 0]); % Link 2
L(3) = Link([0 a3 d3 0]); % Link 3
robot = SerialLink(L, 'name', 'RRR Robot');

q = [0,0,0];
robot.teach(q);