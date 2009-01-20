function trq = delta_torque(robj, q, qp, qpp, dq, dqp, dqpp);
%   Syntax
%       
%       [trq, del_trq] = delta_torque(robj, q, qp, qpp, dq, dqp, dqpp);
%
%   Description
% 
%   Murray and Neuman [13] have developed an efficient recursive 
%   linearized Newton-Euler formulation that can be used to compute 
%   (see appendix A)
%
%       del_trq = D(q)*del_qpp + S1(q,qp)*del_qp + S2(q,qp,qpp)*del_qpp

toMex = createStructForMex(robj);

trq = mex_delta_torque(toMex, q, qp, qpp, dq, dqp, dqpp);