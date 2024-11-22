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

    % Compute theta2 and theta3 (revolute joint variables)
    theta2 = atan2(y, x) - acos(cos_value);
    theta3 = 2 * acos(cos_value) - theta2;

    % Return the joint variables
    q = [d1, theta2, theta3];
end