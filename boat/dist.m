function cost = f(x, u, e, data, obs_x, obs_y, goal, block_int)
    
    num_states = size(x);
    num_states = num_states(2);
    
    state_centers = (x(1:block_int:end, 1:num_states/2) + x(1:block_int:end, (num_states/2)+1:end))/2;

    % distance to goal
    norms = vecnorm(state_centers - goal, 2, 2);
    cost = -sum(norms); % we subtract because we want higher distance from goal to be punished

end