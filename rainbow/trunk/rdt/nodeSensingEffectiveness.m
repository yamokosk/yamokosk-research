function eff = nodeSensingEffectiveness(node, Tbar, Tvar, neval)

fun = @(y)neval(node, y);

% Using unscented transform
% calculate my own weights
% alpha = 1e-3;
% beta = 2;
% K = -9;
% N = length(Tbar);
% Wm = zeros(N,1);
% Wc = zeros(N,1);
% c = N + K;
% X = ut_sigmas(Tbar,Tvar,c);
% for n = 1:(2*N+1)
%     if n == 1
%         W0 = K/c;
%         Wm(n) = W0/alpha^2 + (1/alpha^2) - 1;
%         Wc(n) = W0 + 1 + beta - alpha^2;
%     else
%         Wi = 1/(2*c);
%         Wm(n) = Wi/alpha^2;
%         Wc(n) = Wm(n);
%         X(:,n) = X(:,1) + alpha*( X(:,n) - X(:,1) );
%     end
% end
%[mu,S,C,X,Y,w] = ut_transform(Tbar,Tvar,fun,[],10,0,-9,0,X,{Wm,Wc,c});
[mu,S,C,X,Y,w] = ut_transform(Tbar,Tvar,fun,[],0.85,0,-9,1);
eff = mu;

% Using monte-carlo method
% N = 1000;
% Tbarm = repmat(Tbar,1,N);
% Tstd = sqrt(diag(Tvar));
% Tstdm = repmat(Tstd,1,N);
% Trand = Tbarm + Tstdm .* randn(length(Tbar),N);
% muv = zeros(N,1);
% 
% for n = 1:N
%     muv(n) = fun(Trand(:,n));
% end
% mu = mean(muv);
% S = std(muv);
% eff = S/mu;