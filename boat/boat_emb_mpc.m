function xdot = f(x, u, obs_x, obs_y, mean_flow)
    % x = [v, psi, x, y]
    % u = [thrust, rudder position]   
    xhat = x(5:end);
    x = x(1:4);
   
    uhat = u;
    
    % state-based disturbance
    [wx, whatx] = disturbance_bounds_fitparam(x(3), xhat(3), obs_x);
    [wy, whaty] = disturbance_bounds_fitparam(x(4), xhat(4), obs_y);
    
    w = [wx; wy];
    what = [whatx; whaty];
    
    xdot = [d(x, u, w, xhat, uhat, what); d(xhat, uhat, what, x, u, w)];
    
    % known function is dynamics, affected by river flow in x and y
    function out = d(x, u, w, xhat, uhat, what)

        v = x(1);
        vhat = xhat(1);
        psi = x(2);
        psihat = xhat(2);
        

        out(1) = -v + u(1);
        dpsi_vec = 0.15 .* u(2) .* [v, vhat];
        if v <= vhat
            out(2) = min(dpsi_vec);
        else
            out(2) = max(dpsi_vec);
        end
        out(3) = d_w1w2([v; d_cos(psi, psihat)], [vhat; d_cos(psihat, psi)]) + w(1);
        out(4) = -d_w1w2([vhat; d_sin(psihat, psi)], [v; d_sin(psi, psihat)]) + w(2) - mean_flow;

        out = out';
    end
end

function out = d_w1w2(w, what)
    % assuming 2d input
    values = [w(1)*w(2),
              w(1)*what(2),
              what(1)*what(2),
              what(1)*w(2)];
    % only check velocity term
    if w(1) <= what(1)
        out = min(values);
    else
        out = max(values);
    end
    
%     % check both terms
%     if all(w <= what)
%         out = min(values);
%     elseif all(what <= w)
%         out = max(values);
%     else
%         error("w (%s) and what (%s) are not component-wise comparable", mat2str(w), mat2str(what))
%     end
end

function [wmin, wmax] = disturbance_bounds_fitparam(x, xhat, obs)
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
    
    sigma_confidence = 3;
    [fbs, cfs] = fit_params(obs(:, 1:end-1), obs(:, end), x_space);
    sfs = sqrt(diag(cfs));
    
    
    wmin = min(fbs - (sigma_confidence * sfs));
    wmax = max(fbs + (sigma_confidence * sfs));
    
    if isempty(wmin) || isempty(wmax)
        error('wmin or wmax empty');
    end
end