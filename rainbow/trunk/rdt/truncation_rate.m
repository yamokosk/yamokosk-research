function R = truncation_rate(f,T)
N = length(f);
S = compute_cdf(f);
I = find( S > (1-T)*N );
R = zeros(N,1);
R(I) = (1/T);