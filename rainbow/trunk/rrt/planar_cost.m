function [endpoint,integrand] = planar_cost(sol)
endpoint  = 0;

x1 = sol.states(:,1);
x2 = sol.states(:,2);
x3 = sol.states(:,3);
x4 = sol.states(:,4);
x5 = sol.states(:,5);
x6 = sol.states(:,6);

u1 = sol.controls(:,1);
u2 = sol.controls(:,2);
u3 = sol.controls(:,3);
u4 = sol.controls(:,4);
u5 = sol.controls(:,5);
u6 = sol.controls(:,6);

integrand = 0.5*(x1.^2+x2.^2+x3.^2+x4.^2+x5.^2+x6.^2+u1.^2+u2.^2+u3.^2+u4.^2+u5.^2+u6.^2);