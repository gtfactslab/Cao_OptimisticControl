function disturbance = disturbance_y_obs(z, noise_observations)
    [disturbance, ~] = fit_params(noise_observations(:, 1), noise_observations(:, 2), z);

end