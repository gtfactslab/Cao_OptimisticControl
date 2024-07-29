function xdot = f(t, x, u, dist_y_fn, dist_z_fn)
    % x = [y, vy, z, vz, theta, omega]
    % u = [thrust, roll angular acceleration]
    y = x(1);
    vy = x(2);
    z = x(3);
    vz = x(4);
    theta = x(5);
    omega = x(6);
    
    % gravity
    g = 9.81;
    
    % disturbance is completely altitude-based for now
    xdot(1) = vy;
    xdot(2) = -u(1)*sin(theta) + dist_y_fn(z);
    xdot(3) = vz;
    xdot(4) = u(1)*cos(theta) - g + dist_z_fn(z);
    xdot(5) = omega;
    xdot(6) = u(2);
    
    xdot = xdot';
end