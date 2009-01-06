function xi = lp(x0, xf, udata)

Ni = 3;
xi = zeros(size(x0,1),Ni);
alpha = linspace(0,1,Ni+2);
for n = 2:Ni+1
    xi(:,n-1) = alpha(n)*xf + x0;
end