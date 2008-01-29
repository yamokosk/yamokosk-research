function [x, w] = gauss_points(n) 
% function returns weights and points for gauss quadrature
x = legroots(n);
Pnp1 = legendre(n+1,x); 
Pnp1 = Pnp1(1,:)'; 
Pndot = -(n+1)./(1-x.^2).*Pnp1; 
w = 1./(Pndot).^2.*(2./(1-x.^2)); 