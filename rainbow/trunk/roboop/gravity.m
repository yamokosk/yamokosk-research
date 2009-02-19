function G = inertia(robj, q)
%   Syntax
%       
%       G = inertia(robj, q)
%
%   Description
%
%   Returns the manipulator gravity vector.

toMex = createStructForMex(robj);

G = mex_gravity(toMex, q);