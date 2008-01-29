function G = grad(func, x, opts)
f = func(x,opts.userdata);
dim = length(f);

% if len > 1
%     error('Assumes a scalar real valued function.');
% end

D = genD(func, x, opts);

% if (numrows(D) ~= 1) 
%     error('BUG! Should not get here');
% end
G = D(:,length(x));