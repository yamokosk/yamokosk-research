function M = inertia(robj, q)
%   Syntax
%       
%       M = inertia(robj, q)
%
%   Description
%
%   Returns the manipulator mass matrix.

toMex = createStructForMex(robj);

M = mex_inertia(toMex, q);