function disturbance = disturbance_y(y)
    noise_observations = [-3, 2;
                            0, 1;
                            2, -2;
                            5, -2;
                            -5, -3;
                            7, -1;
                            9, 1;
                            11, 3;
                            13, 2];
    [disturbance, ~] = fit_params(noise_observations(:, 1), noise_observations(:, 2), y);
end