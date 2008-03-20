function [endpoint,integrand] = planar_cost(sol)
endpoint  = 0;

q_min = [-64, -107, -165] * (pi/180);
q_max = [124, 158, 165] * (pi/180);
qp_min = -1*[57, 114, 360] * (pi/180);
qp_max = [57, 114, 360] * (pi/180);
u_min = -1*[4.64, 2.0, 0.29]*50;
u_max = [4.64, 2.0, 0.29]*50;

q11 = (2*sol.states(:,1) - (q_max(1) + q_min(1))) ./ (q_max(1) - q_min(1));
q12 = (2*sol.states(:,2) - (q_max(2) + q_min(2))) ./ (q_max(2) - q_min(2));;
q13 = (2*sol.states(:,3) - (q_max(3) + q_min(3))) ./ (q_max(3) - q_min(3));;
qp11 = (2*sol.states(:,4) - (qp_max(1) + qp_min(1))) ./ (qp_max(1) - qp_min(1));
qp12 = (2*sol.states(:,5) - (qp_max(2) + qp_min(2))) ./ (qp_max(2) - qp_min(2));
qp13 = (2*sol.states(:,6) - (qp_max(3) + qp_min(3))) ./ (qp_max(3) - qp_min(3));

q21 = (2*sol.states(:,7) - (q_max(1) + q_min(1))) ./ (q_max(1) - q_min(1));
q22 = (2*sol.states(:,8) - (q_max(2) + q_min(2))) ./ (q_max(2) - q_min(2));;
q23 = (2*sol.states(:,9) - (q_max(3) + q_min(3))) ./ (q_max(3) - q_min(3));;
qp21 = (2*sol.states(:,10) - (qp_max(1) + qp_min(1))) ./ (qp_max(1) - qp_min(1));
qp22 = (2*sol.states(:,11) - (qp_max(2) + qp_min(2))) ./ (qp_max(2) - qp_min(2));
qp23 = (2*sol.states(:,12) - (qp_max(3) + qp_min(3))) ./ (qp_max(3) - qp_min(3));

u1 = (2*sol.controls(:,1) - (u_max(1) + u_min(1))) ./ (u_max(1) - u_min(1));
u2 = (2*sol.controls(:,2) - (u_max(2) + u_min(2))) ./ (u_max(2) - u_min(2));
u3 = (2*sol.controls(:,3) - (u_max(3) + u_min(3))) ./ (u_max(3) - u_min(3));
u4 = (2*sol.controls(:,4) - (u_max(1) + u_min(1))) ./ (u_max(1) - u_min(1));
u5 = (2*sol.controls(:,5) - (u_max(2) + u_min(2))) ./ (u_max(2) - u_min(2));
u6 = (2*sol.controls(:,6) - (u_max(3) + u_min(3))) ./ (u_max(3) - u_min(3));

% Penalize joint angles and joint torque
% integrand = 0.5*(x1.^2 + x2.^2 + x3.^2 + ...
%                  x7.^2 + x8.^2 + x9.^2 + ...
%                  u1.^2 + u2.^2 + u3.^2 + u4.^2 + u5.^2 + u6.^2);

% Penalize joint angles, joint velocities and joint torque
% integrand = 0.5*(q1.^2 + q2.^2 + q3.^2 + qp1.^2 + qp2.^2 + qp3.^2 + ...
%                  u1.^2 + u2.^2 + u3.^2);

integrand = 0.5*(q11.^2 + q12.^2 + q13.^2 + qp11.^2 + qp12.^2 + qp13.^2 + ...
                 q21.^2 + q22.^2 + q23.^2 + qp21.^2 + qp22.^2 + qp23.^2 + ...
                  u1.^2 + u2.^2 + u3.^2 + u4.^2 + u5.^2 + u6.^2);