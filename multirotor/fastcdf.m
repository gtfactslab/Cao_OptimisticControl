% faster way of calculating cdf
function out = fastcdf(x)
    out = 0.5*erfc(-x./sqrt(2));
    %out = phi(x); %requires mex file, close in calc time though
end