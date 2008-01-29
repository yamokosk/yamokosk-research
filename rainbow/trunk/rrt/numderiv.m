function [G, H] = numderiv(func, x, opts)

f = func(x,opts.userdata);
dim = length(f);

D = genD(func, x, opts);
G = D(:,1:length(x));

H = zeros(dim,length(x),length(x));

for d = 1:dim
    u = length(x);
    for i = 1:length(x)
        for j = 1:i
            u = u + 1;
            H(d,i,j) = D(d,u);
        end
    end

    H(d,:,:) = squeeze(H(d,:,:)) + squeeze(H(d,:,:))';
    for n = 1:length(x)
        H(d,n,n) = H(d,n,n)/2;
    end
end