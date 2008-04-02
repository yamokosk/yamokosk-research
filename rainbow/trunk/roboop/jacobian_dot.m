function J_dot = jacobian_dot(robj, q, q_dot, ref)
%   Syntax
% 
%       J_dot = jacobian_dot(robj, q, q_dot, ref);
%
%   Description
%
%   The manipulator Jacobian time derivative can be used to compute the 
%   end effector acceleration due to joints velocities [9]:
%
%       xi_dbldot = Ji_dot(q, q_dot) * q_dot

if nargin < 4
	ref = 0;
end

mat = mat_from_struct(robj);

J_dot = mex_jacobian_dot(mat, q, q_dot, ref);