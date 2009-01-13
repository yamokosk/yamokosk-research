function xi = lp(x0, xf, udata)

Ni = 5;
xi = zeros(size(x0,1),Ni);
alpha = linspace(0,1,Ni);
for n = 1:Ni
    xi(:,n) = (x0 + alpha(n) * (xf - x0));
end

xi = xi(:,2:end-1);