function disturbance = disturbance_x(x)
    noise_observations = [10, -3;
                            5, -1;
                            2, 2;
                            0, 3;
                            -2, 0;
                            -7, 1;
                            -10, -2];
    [disturbance, ~] = fit_params(noise_observations(:, 1), noise_observations(:, 2), x);
end
