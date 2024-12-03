function q = inverse_kinematics_prr(x, y, z, link_length)
    % Compute d1 (prismatic joint variable)
    d1 = z;

    % Compute the distance in the XY plane
    r = sqrt(x^2 + y^2);

    % Check if the target is reachable
    if r > 2 * link_length
        error('Target is out of reach!');
    end

    % Clamp values for acos to avoid complex numbers
    cos_value = max(-1, min(1, link_length / r));

    % Compute intermediate angles
    phi = atan2(y, x);               % Angle of the target position in the XY plane
    delta = acos(cos_value);         % Angle subtended by the link at the target

    % Compute theta2 and theta3
    theta2 = phi - delta;            % First joint angle (relative to horizontal)
    theta3 = 2 * delta;              % Second joint angle (closing the triangle)

    % Return the joint variables
    q = [d1, theta2, theta3];
end