%% Gaussian Process functions
function [f_bar_star, cov_f_star] = f(x, y, x_star)

    % Inverted Pendulum
%     sigma_f = 80; % 80 for radial, 10 for matern
%     l = 1.5;
%     sigma_n = 0.1;
%     
%     % Kinematic Bike
%     sigma_f = 60000;
%     l = 250;
%     sigma_n = 1;
    
%     % Kinematic Bike with icy state
%     sigma_f = 0.01;
%     l = 1.5;
%     sigma_n = 0.001;
    
    % quad in wind field
    sigma_f = 3;
    l = 2;
    sigma_n = 0.01;
%     
    [K, K_star2, K_star] = cov_matrices(x, x_star, sigma_f, l);
    [f_bar_star, cov_f_star] = gpr_params(y, K, K_star2, K_star, sigma_n);
end


function kernel =  kernel(x, y, sigma_f, l)
    % check dimension of x
    x_shape = size(x);
    if x_shape(2) == 1 % if observation points are 1D
        kernel = sigma_f * exp(-1*((x-y)'.^2) / (2*l^2));
    else
    % squared exponential aka radial basis kernel
        kernel = sigma_f * exp(-1*sum((x-y)'.^2) / (2*l^2));
    end
    % sum((x - y)'.^2) returns row-wise sum
    
    % matern 3/2 function
%     r = sqrt(sum((x - y)'.^2));
%     kernel = sigma_f^2 * (1 + sqrt(3)*r/l)*exp(-sqrt(3)*r/l);
    
    % matern 5/2 function
%     r = sqrt(sum((x - y)'.^2));
%     kernel = sigma_f^2 * (1 + sqrt(5)*r/l)*exp(-sqrt(5)*r/l);
end

function [K, K_star2, K_star] = cov_matrices(x, x_star, sigma_f, l)
    x_size = size(x);
    xs_size = size(x_star);
    K = zeros(x_size(1));
    K_star2 = zeros(xs_size(1));
    K_star = zeros([xs_size(1), x_size(1)]);
    
%     for i = 1:x_size(1)
%         for j = 1:x_size(1)
%             K(i, j) = kernel(x(i, :)', x(j, :)', sigma_f, l);
%         end
%     end
    
%     for i = 1:xs_size(1)
%         for j = 1:xs_size(1)
%             K_star2(i, j) = kernel(x_star(i, :)', x_star(j, :)', sigma_f, l);
%         end
%     end

%     for i = 1:xs_size(1)
%         for j = 1:x_size(1)
%             K_star(i, j) = kernel(x_star(i, :)', x(j, :)', sigma_f, l);
%         end
%     end
    

    for i = 1:x_size(1)
        K(i, :) = kernel(x(i, :), x, sigma_f, l);
    end
    
    for i = 1:xs_size(1)
        K_star2(i, :) = kernel(x_star(i, :), x_star, sigma_f, l);
    end

    for i = 1:xs_size(1)
        K_star(i, :) = kernel(x_star(i, :), x, sigma_f, l);
    end
    

end

function [f_bar_star, cov_f_star] = gpr_params(y, K, K_star2, K_star, sigma_n)
    n = size(K);
    n = n(1);
    
    % calculate cholesky decomp
    L = chol(K + ((sigma_n^2) * eye(n)));
    alpha = L \ (L' \ y);

    % mean
    f_bar_star = K_star * alpha;
    
    %covariance
    v = (L' \ K_star')';
    cov_f_star = K_star2 - (v * v');
   
end
