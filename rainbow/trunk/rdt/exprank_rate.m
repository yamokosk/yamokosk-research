% Tournament reproduction rate
function R = exprank_rate(f,alpha)
% alpha is ratio of reproduction rate of the worst and best individuals.
% Interesting ranges of alpha are 1e-20 to 1;
N = length(f);
S = compute_cdf(f);
R = ((alpha * log(alpha))/(alpha-1))*alpha.^(-S/N);