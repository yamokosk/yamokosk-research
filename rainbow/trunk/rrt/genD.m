function D = genD(func, x, opts)


% v - reduction factor for Richardson iterations. This could be a parameter
% but the way the formula is coded it is assumed to be 2.

% defaults
eps = 1e-4;
d = 0.0001;
r = 4;
v = 2;

if (v ~= 2)
    error('The current code assumes v is 2 (the default).');
end

% f0 is the value of the function at x
f0 = func(x, opts.userdata);

p = length(x); % number of parameters (theta)
h0 = abs(d*x) + eps*(x==0.0);
D = zeros(length(f0), (p * (p + 3))/2);

% length(f0) is the dim of the sample space
% (p * (p + 3))/2 is the number of columns of matrix D (first der. & lower
% triangle of Hessian)
Daprox = zeros(length(f0),r);
Hdiag = zeros(length(f0), p);
Haprox = zeros(length(f0), r);
vec = [1:p]';

for i = 1:p % each parameter - first deriv. % hessian diagonal
    h = h0;
    for k = 1:r % successively reduce h
        f1 = func(x + (vec == i)*h(i), opts.userdata);
        f2 = func(x - (vec == i)*h(i), opts.userdata);

        Daprox(:,k) = (f1 - f2) / (2 * h(i));      % F'(i)
        Haprox(:,k) = (f1 - 2*f0 + f2) / h(i)^2;   % F''(i) hessian diagonal

        h = h / v; % Reduce h by 1/v;
    end

    for m = 1:r-1
        for k = 1:r-m
            Daprox(:,k) = ( Daprox(:,k+1)*(4^m) - Daprox(:,k) )/(4^m - 1);
            Haprox(:,k) = ( Haprox(:,k+1)*(4^m) - Haprox(:,k) )/(4^m - 1);
        end
    end

    D(:,i) = Daprox(:,1);
    Hdiag(:,i) = Haprox(:,1);
end

u = p;

for i = 1:p   % 2nd derivative - do lower half of hessian only
    for j = 1:i
        u = u + 1;
        if (i == j)
            D(:,u) = Hdiag(:,i);
        else
            h = h0;
            for k = 1:r % successively reduce h
                f1 = func(x + (vec == i)*h(i) + (vec == j)*h(j), opts.userdata);
                f2 = func(x - (vec == i)*h(i) - (vec == j)*h(j), opts.userdata);
                Daprox(:,k) = (f1 - 2*f0 + f2 - ...
                    Hdiag(:,i)*h(i)^2 - ...
                    Hdiag(:,j)*h(j)^2) / (2*h(i)*h(j));  % F''(i,j)
                h = h/v;    % Reduced h by 1/v
            end

            for m = 1:r-1
                for k = 1:r-m
                    Daprox(:,k) = (Daprox(:,k+1)*(4^m) - Daprox(:,k))/(4^m-1);
                end
            end

            D(:,u) = Daprox(:,1);
        end
    end
end
 