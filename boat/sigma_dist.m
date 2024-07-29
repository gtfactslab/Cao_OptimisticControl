function cost = f(x, u, e, data, obs_x, obs_y, goal, blocking_interval)
     
    state_centers = (x(1:blocking_interval:end, 1:4) + x(1:blocking_interval:end, 5:end))/2;
    
    [~, cov_f_star_x] = fit_params(obs_x(:, 1:end-1), obs_x(:, end), state_centers(:, 3));
    [~, cov_f_star_y] = fit_params(obs_y(:, 1:end-1), obs_y(:, end), state_centers(:, 4));
    
    std_f_star_x = sqrt(diag(cov_f_star_x));
    std_f_star_y = sqrt(diag(cov_f_star_y));
    
    cost_x = sum(std_f_star_x);
    cost_y = sum(std_f_star_y);
    
    % distance to goal
    norms_x = abs(state_centers(:, 3) - goal(3)).*exp(-cost_x);
    norms_y = abs(state_centers(:, 4) - goal(4)).*exp(-cost_y);
    
    cost = sum(cost_x-norms_x) + 0.25*sum(cost_y-norms_y); % we subtract because we want higher distance from goal to be punished

end