function H = hessian(func, x, opts)

f = func(x,opts.userdata);
dim = length(f);

% if len > 1
%     error('Assumes a scalar real valued function.');
% end

D = genD(func, x, opts);

% if (numrows(D) ~= 1) 
%     error('BUG! Should not get here');
% end

H = zeros(length(x),length(x),dim);

for d = 1:dim
    u = length(x);
    for i = 1:length(x)
        for j = 1:i
            u = u + 1;
            H(i,j,d) = D(d,u);
        end
    end

    H(:,:,d) = H(:,:,d) + H(:,:,d)';
    for n = 1:size(H,1)
        H(n,n,d) = H(n,n,d)/2;
    end
end