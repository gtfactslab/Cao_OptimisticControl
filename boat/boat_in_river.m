function xdot = f(t, x, u, mean_flow)

    % x = [v, psi, x, y]
    % u = [thrust, rudder angle]
    v = x(1);
    psi = x(2);
    
    xdot(1) = -v + u(1);
    xdot(2) = 0.15 * v * u(2);
    xdot(3) = v * cos(psi) + disturbance_x(x(3));
    xdot(4) = -v * sin(psi) - mean_flow + disturbance_y(x(4));

    xdot = xdot';
end