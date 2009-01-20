function trq = torque(robj, q, qp, qpp, Fext, Next);
%   Syntax
%       
%       trq = torque(robj, q, qp, qpp);
%       trq = torque(robj, q, qp, qpp, Fext, Next);
%
%   Description
% 
%   This function computes tau from q, qp and qpp which is the inverse 
%   dynamics problem. The recursive Newton-Euler (RNE) formulation is one 
%   of the most computationally efficient algorithm [12, 13] used to solve 
%   this problem (see appendix A). The second form allows the inclusion 
%   the contribution of a load applied at the last link.

if nargin < 5
    Fext = zeros(3,1);
	Next = zeros(3,1);
end

toMex = createStructForMex(robj);

trq = mex_torque(toMex, q, qp, qpp, Fext, Next);