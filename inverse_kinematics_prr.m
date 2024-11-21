function q = inverse_kinematics_prr(x, y, z, link_length)
    % inverse_kinematics_prr: Calculate joint variables for a PRR robot
    %
    % Inputs:
    % - x, y, z: Target end-effector position
    % - link_length: Length of the second link (assumed constant)
    %
    % Outputs:
    % - q: Vector of joint variables [d1, theta2, theta3]

    % Prismatic joint (d1): Accounts for the z position directly
    d1 = z; % The z position is controlled by the prismatic joint

    % Planar motion (x, y): Solve for theta2 and theta3 using geometry
    r = sqrt(x^2 + y^2); % Radial distance in the x-y plane
    if r > 2 * link_length
        error('Target position is out of the robot workspace.');
    end

    % Using the law of cosines to calculate theta2 and theta3
    cos_theta3 = (r^2 - 2 * link_length^2) / (2 * link_length^2);
    theta3 = acos(cos_theta3); % Theta3 has two possible solutions: ±acos

    theta2 = atan2(y, x) - atan2(link_length * sin(theta3), link_length + link_length * cos(theta3));

    % Combine results
    q = [d1, theta2, theta3];
end
