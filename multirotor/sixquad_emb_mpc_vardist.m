function xdot = f(x, u, obs_y, obs_z)
    % x = [y, vy, z, vz, theta, omega]
    % u = [thrust, roll angular acceleration]   
    xhat = x(7:end);
    x = x(1:6);
    
    sig_lower = u(end-1);
    sig_upper = u(end);
    
    input_uncertainty = 0;
    uhat = u(1:end-2) + input_uncertainty;
    u = u(1:end-2) - input_uncertainty;
    
    % gravity
    g = 9.81;
    
    % disturbance solely based on altitude
    [wy, whaty] = disturbance_bounds_fitparam(x(3), xhat(3), obs_y, sig_upper, sig_lower);
    [wz, whatz] = disturbance_bounds_fitparam(x(3), xhat(3), obs_z, sig_upper, sig_lower);
    
%     % disturbance based on altitude and horiz pos
%     [wy, whaty] = disturbance_bounds_fitparam([x(1); x(3)], [xhat(1); xhat(3)], obs_y);
%     [wz, whatz] = disturbance_bounds_fitparam([x(1); x(3)], [xhat(1); xhat(3)], obs_z);
    
    w = [wy; wz];
    what = [whaty; whatz];
    
    xdot = [d(x, u, w, xhat, uhat, what); d(xhat, uhat, what, x, u, w)];
    
    % known function is dynamics, affected by wind disturbance in y and z
    function out = d(x, u, w, xhat, uhat, what)

        vy = x(2);
        vz = x(4);
        theta = x(5);
        omega = x(6);

        thetahat = xhat(5);

        out(1) = vy;
        out(2) = -d_w1w2([uhat(1), d_sin(thetahat, theta)],[u(1), d_sin(theta, thetahat)]) + w(1);
        out(3) = vz;
        out(4) = d_w1w2([u(1), d_cos(theta, thetahat)], [uhat(1), d_cos(thetahat, theta)]) - g + w(2);
        out(5) = omega;
        out(6) = u(2);

        out = out';
    end
end

function out = d_w1w2(w, what)
    % assuming 2d input
    values = [w(1)*w(2),
              w(1)*what(2),
              what(1)*what(2),
              what(1)*w(2)];
    if all(w <= what)
        out = min(values);
    elseif all(what <= w)
        out = max(values);
    else
        error("w (%s) and what (%s) are not component-wise comparable", mat2str(w), mat2str(what))
    end
end

function [wmin, wmax] = disturbance_bounds_fitparam(x, xhat, obs, sig_upper, sig_lower)
    % obs is assumed to be a strict list of the observations
    % of the GP
%     sprintf('emb_fitparam')
    x_lower = min(x, xhat);
    x_upper = max(x, xhat);
    spacing = (x_upper - x_lower)./10;

    if numel(x) == 1 %1D case
        x_space = [x_lower, x_lower:spacing:x_upper, x_upper]';
    elseif numel(x) == 2 %2D case
        [x1_vec, x2_vec] = meshgrid([x_lower(1), x_lower(1):spacing(1):x_upper(1), x_upper(1)], [x_lower(2), x_lower(2):spacing(2):x_upper(2), x_upper(2)]);
        x_space = [reshape(x1_vec, [], 1), reshape(x2_vec, [], 1)];
    else
        error("disturbance can only take 1d or 2d input")
    end
    
    [fbs, cfs] = fit_params(obs(:, 1:end-1), obs(:, end), x_space);
    sfs = sqrt(diag(cfs));
    
    
    wmin = min(fbs - (sig_lower * sfs));
    wmax = max(fbs + (sig_upper * sfs));
    
    if isempty(wmin) || isempty(wmax)
        error('wmin or wmax empty');
    end
end