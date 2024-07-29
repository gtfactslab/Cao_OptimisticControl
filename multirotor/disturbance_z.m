function disturbance = disturbance_z(z)
    noise_observations = [10, 0;
                            8, 3;
                            5, 1;
                            2, -1;
                            0, 0;
                            -2, -1;
                            -5, -2];
    [disturbance, ~] = fit_params(noise_observations(:, 1), noise_observations(:, 2), z);
end
