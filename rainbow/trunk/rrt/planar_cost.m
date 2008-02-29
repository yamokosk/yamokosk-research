function [endpoint,integrand] = planar_cost(sol)
endpoint  = 0;

x1 = sol.states(:,1); % q1
x2 = sol.states(:,2); % q2
x3 = sol.states(:,3); % q3

x4 = sol.states(:,4); % qp1
x5 = sol.states(:,5); % qp2
x6 = sol.states(:,6); % qp3

x7 = sol.states(:,7); % q1
x8 = sol.states(:,8); % q2
x9 = sol.states(:,9); % q3

x10 = sol.states(:,10); % qp1
x11 = sol.states(:,11); % qp2
x12 = sol.states(:,12); % qp3

u1 = sol.controls(:,1);
u2 = sol.controls(:,2);
u3 = sol.controls(:,3);

u4 = sol.controls(:,4);
u5 = sol.controls(:,5);
u6 = sol.controls(:,6);

% Penalize joint angles and joint torque
% integrand = 0.5*(x1.^2 + x2.^2 + x3.^2 + ...
%                  x7.^2 + x8.^2 + x9.^2 + ...
%                  u1.^2 + u2.^2 + u3.^2 + u4.^2 + u5.^2 + u6.^2);

% Penalize joint angles, joint velocities and joint torque
integrand = 0.5*(x1.^2 + x2.^2 + x3.^2 + x4.^2 + x5.^2 + x6.^2 + ...
                 x7.^2 + x8.^2 + x9.^2 + x10.^2 + x11.^2 + x12.^2 + ...
                 u1.^2 + u2.^2 + u3.^2 + u4.^2 + u5.^2 + u6.^2);