% Constraints function
function c = con_min_tf(z,Prob)
t0 = Prob.user.t0;
x0 = Prob.user.x0;
xf = Prob.user.xf;
dim = Prob.user.dim;
N = length(Prob.user.w);
nstates = dim*2;

% Unpack the data
[tf,U,X] = unpack(z,dim,N);

% Scale diff matrices by time scale
Db = Prob.user.Dbar;
D = Prob.user.D;
w = Prob.user.w;

% Allocate space for constraints
CEQ = zeros(nstates, N + 1);
fi = zeros(nstates,N);

% Compute dynamic constraints
for i = 1:N
    % Compute RHS of ODE at Xi, Ui, ti
    fi(:,i) = Prob.user.odefun(X(:,i),U(:,i));

    % Compute sum_k ( D(i,k) * X(tk) )
    Dik_mat = repmat(D(i,:), nstates, 1);
    sum_k_DikXk = sum(Dik_mat .* X, 2);

    % Compute ith dynamic constraint: 
    %   2/(tf - t0) * (D_bar(i) * X(t0) + sum_k ( D(i,k) * X(tk) )) = f(X(ti), U(ti), ti)
    CEQ(:,i) = 2/(tf - t0) * ( Db(i)*x0 + sum_k_DikXk ) - fi(:,i);
end

% Compute sum_k ( w(k) * f(X(tk), U(tk), tk) )
Wk_mat = repmat(w', nstates, 1);
sum_k_WkFk = sum(Wk_mat .* fi, 2);

% Compute final time/state constraint:
%   X(tf) - X(t0) = (tf - t0)/2 * sum_k ( w(k) * f(X(tk), U(tk), tk) )
CEQ(:,end) = xf - x0 - (tf - t0)/2 * sum_k_WkFk;

% Put constraints into linear array
c = reshape(CEQ, nstates*(N+1), 1);
end