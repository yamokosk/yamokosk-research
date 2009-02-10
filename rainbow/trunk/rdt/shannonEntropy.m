function H = shannonEntropy(P,logfun)

if nargin < 2
    logfun = @log;
end

H = -sum( P .* logfun(P) );