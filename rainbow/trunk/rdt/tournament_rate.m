% Tournament reproduction rate
function R = tournament_rate(f,t)
%R = t * (S(f)/N)^(t-1);
N = length(f);
S = compute_cdf(f);
R = t * (S/N).^(t-1);