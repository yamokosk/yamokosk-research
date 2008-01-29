function [Dg, Dbar, x, w] = legdiff_G(n)
% Function generates the Gauss collocation points and weights (x, w) and the
% Differential approximation matrix D, bar D
% Gauss Pts
[x, w] = gauss_points(n);
% Add initial point -1
x = [x; -1];
x = sort(x);
n = n+1;
% Eval derivative of Lagrange polynomials
for j = 1:n
    for i = 1:n;
        prod = 1;
        sum = 0;
        if j == i
            for k = 1:n
                if k~=i
                    sum = sum+1/(x(i)-x(k));
                end
            end
            D(i,j) = sum;
        else
            for k = 1:n
                if (k~=i)&(k~=j)
                    prod = prod * (x(i)-x(k));
                end
            end
            for k = 1:n
                if k~=j
                    prod = prod/(x(j)-x(k));
                end
            end
            D(i,j) = prod;
        end
    end
end
Dg = D(2:end,2:end);
Dbar = D(2:end,1);
x = x(2:end);


