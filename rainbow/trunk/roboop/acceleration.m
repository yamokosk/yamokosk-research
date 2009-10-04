function qpp = acceleration(robj, q, qp, tau, Fext, Next);
%   Syntax
%       
%       trq = torque(robj, q, qp, tau);
%       trq = torque(robj, q, qp, tau, Fext, Next);
%
%   Description
% 
%   This function computes qpp from q, qp and tau which is the forward 
%   dynamics problem. 

if nargin < 5
    Fext = zeros(3,1);
	Next = zeros(3,1);
end

toMex = createStructForMex(robj);

qpp = mex_acceleration(toMex, q, qp, tau, Fext, Next);