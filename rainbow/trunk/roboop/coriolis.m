function C = coriolis(robj, q, qp, pp)
%   Syntax
%       
%       C = coriolis(robj, q, qp)
%
%   Description
%
%   Returns the manipulator Coriolis vector.

toMex = createStructForMex(robj);

C = mex_coriolis(toMex, q, qp, pp);