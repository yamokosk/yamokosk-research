function [A, x, w] = legint(n) 
% Function generates the Gauss collocation points and weights (x, w) and the
% integration approximation matrix A
[x, w] = gauss_points(n);
if n == 1
    A = w/2;
    return;
end
g = 1;
Pn = zeros(n+1,n);
for v = 0:n
    P = legendre(v,x);
    if v == 0
        P =P';
    end
    Pn(v+1,:) = P(1,:);
end
% loop through i,k
for i = 1:n;
    for k = 1:n;
        SUMP = 0;
        for v = 1:n-2
            SUMP = SUMP + Pn(v+1,k)*(Pn(v+2,i) - Pn(v,i));
        end
        A(i,k) = w(k)/2*(1+x(i) + SUMP + g*Pn(n,k)*(Pn(n+1,i) - Pn(n-1,i)));
    end
end
