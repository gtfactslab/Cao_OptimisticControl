function cost = f(x, u, e, data, obs_x, obs_y, goal, blocking_interval)
     
    state_centers = (x(1:blocking_interval:end, 1:4) + x(1:blocking_interval:end, 5:end))/2;
    
    [~, cov_f_star_x] = fit_params(obs_x(:, 1:end-1), obs_x(:, end), state_centers(:, 3));
    [~, cov_f_star_y] = fit_params(obs_y(:, 1:end-1), obs_y(:, end), state_centers(:, 4));
    
    std_f_star_x = sqrt(diag(cov_f_star_x));
    std_f_star_y = sqrt(diag(cov_f_star_y));
    
    cost_x = sum(std_f_star_x);
    cost_y = sum(std_f_star_y);
    
    cost = sum(cost_x) + 0.5*sum(cost_y);

end