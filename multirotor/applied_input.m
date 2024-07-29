function cost = f(x, u, e, data, block_int)
    
    inputs = u(1:block_int:end-1, 1:2);

    % sum of magnitude of inputs
%     cost = sum(sqrt(sum(inputs.^2)));
    
    % sum of absolute value
%     cost = sum(sum(abs(inputs)));
    
    % sum of forces
    L = 1/2;
    % thrust = F1 + F2
    % angular moment = L/2 * (F1 - F2)
    A = [1, 1; L/2, -L/2];
    b = inputs';
    F = A \ b;
    cost = sum(sum(abs(F)));
    
%     % toggle: weight by probability of validity
%     sig_upper = u(1, end);
%     sig_lower = -u(1, end-1);
%     prob = normcdf(sig_upper) - normcdf(sig_lower);
%     cost = cost/prob;
    
end