% Tournament reproduction rate
function R = linearrank_rate(f,eta)
%R = t * (S(f)/N)^(t-1);
N = length(f);
S = compute_cdf(f);
R = eta + 2 * (1-eta)/N * S;