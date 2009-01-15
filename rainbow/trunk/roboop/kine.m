function T = kine(robj, q, link)
%   Syntax
%
%       T = kine(robj, q);
%       T = kine(robj, q, link);
%
%   Description
%
%   Return the orientation and position or the equivalent homogeneous 
%   transform for the last (if not supplied) or the ith link. For example:
%
%       % forward kinematics up to the last link
%       Thomo = kine(robj, q);
%
%       % forward kinematics up to the 2nd link
%       Thomo = kine(robj, q, 2);

if nargin < 3
	link = robj.available_dof;
end

toMex = createStructForMex(robj);

T = mex_kine(toMex, q, link);