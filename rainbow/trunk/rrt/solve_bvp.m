function [tf,U,X] = solve_bvp(t0, x0, xf, odefun)
nstates = length(x0);
dim = nstates/2;

% Create Gauss collocation points
N = 8;
[D, Dbar, x, w] = legdiff_G(N);

% Setup optimization problem
z0 = [1; zeros(dim*N,1); zeros(nstates*N,1)];
lb = [0; -5*ones(dim*N,1); -inf*ones(nstates*N,1)];
ub = [inf; 5*ones(dim*N,1); inf*ones(nstates*N,1)]; 

opts = optimset('Display','iter');
[zopt, f_k] = fmincon(@costfun, z0, [], [], [], [], lb, ub, @confun, opts);
[tf,U,X] = unpack(zopt,dim,N);

%     function J = costfun(z)
%         % Unpack the data
%         %[tf,U] = unpack(z,dim,N);
%         
%         % Compute cost
%         %w_hat = w * (tf-t0)/2;
%         %J = tf + (sum(U.^2)*w_hat);
%         J = tf
%     end

    function [c,ceq] = confun(z)
        % Unpack the data
        [tf,U,X] = unpack(z,dim,N);
        
        % Scale diff matrices by time scale
        Dbar_hat = Dbar * (2/(tf-t0));
        D_hat = D * (2/(tf-t0));
        w_hat = w * (tf-t0)/2;
        
        % Allocate space for constraints
        CEQ = zeros(nstates, N + 1);
        f = zeros(nstates,N);
        c = [];
        
        % Compute constraints
        for n = 1:N
            xD = zeros(nstates,1);
            for r = 1:N
                xD = xD + D_hat(n,r)*X(:,r);
            end
            f(:,n) = odefun(X(:,n),U(:,n));
            CEQ(:,n) = Dbar_hat(n)*x0 - f(:,n) + xD;
        end
        
        CEQ(:,end) = x0 - xf;
        for n = 1:N
            CEQ(:,end) = CEQ(:,end) + w_hat(n) * f(:,n);
        end
        
        ceq = reshape(CEQ, nstates*(N+1), 1);
    end

end


function [tf, U, X] = unpack(z, dim, N)
nstates = dim*2;
tf = z(1); U = reshape(z(2:dim*N + 1),dim,N); X = reshape(z(dim*N + 2:end),nstates,N);
end

function J = costfun(z)
% Unpack the data
%[tf,U] = unpack(z,dim,N);

% Compute cost
%w_hat = w * (tf-t0)/2;
%J = tf + (sum(U.^2)*w_hat);
J = tf
end

        