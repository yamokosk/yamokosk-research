% Constraints function
function c = con_min_effort(y,Prob)
% Unpack data
t0 = Prob.user.t0;
x0 = Prob.user.x0;
tf = Prob.user.tf;
xf = Prob.user.xf;
nu = Prob.user.nu;
ns = Prob.user.ns;
N = Prob.user.N;
w = Prob.user.w;
D = Prob.user.D;
Db = Prob.user.Dbar;

U = reshape(y(1:nu*N),nu,N);
X = reshape(y(nu*N + 1:end), ns, N);

% Allocate space for constraints
CEQ = zeros(ns, N + 1);
fi = zeros(ns,N);

% Compute dynamic constraints
for i = 1:N
    % Compute RHS of ODE at Xi, Ui, ti
    fi(:,i) = Prob.user.odefun(X(:,i),U(:,i));

    % Compute sum_k ( D(i,k) * X(tk) )
    Dik_mat = repmat(D(i,:), ns, 1);
    sum_k_DikXk = sum(Dik_mat .* X, 2);

    % Compute ith dynamic constraint: 
    %   2/(tf - t0) * (D_bar(i) * X(t0) + sum_k ( D(i,k) * X(tk) )) = f(X(ti), U(ti), ti)
    CEQ(:,i) = 2/(tf - t0) * ( Db(i)*x0 + sum_k_DikXk ) - fi(:,i);
end

% Compute sum_k ( w(k) * f(X(tk), U(tk), tk) )
Wk_mat = repmat(w', ns, 1);
sum_k_WkFk = sum(Wk_mat .* fi, 2);

% Compute final time/state constraint:
%   X(tf) - X(t0) = (tf - t0)/2 * sum_k ( w(k) * f(X(tk), U(tk), tk) )
CEQ(:,end) = xf - x0 - (tf - t0)/2 * sum_k_WkFk;

% Put constraints into linear array
c = reshape(CEQ, ns*(N+1), 1);
end