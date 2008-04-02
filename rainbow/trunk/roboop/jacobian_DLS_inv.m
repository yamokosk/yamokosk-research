function J_inv = jacobian_DLS_inv(robj, q, eps, lambda_max, ref)
%   Syntax
% 
%       J_inv = jacobian_DLS_inv(robj, q)
%       J_inv = jacobian_DLS_inv(robj, q, eps)
%       J_inv = jacobian_DLS_inv(robj, q, eps, lambda_max)
%       J_inv = jacobian_DLS_inv(robj, q, eps, lambda_max, ref)
%
%   Description
%       
%   This function returns the inverse Jacobian Matrix for 6 dof 
%   manipulator based on the Damped Least-Squares scheme [10]. Using the 
%   singular value decomposition. A singular region can be selected on the 
%   basis of the smallest singular value of J. Outside the region the 
%   exact solution is returned, while inside the region a 
%   configuration-varying damping factor is introduced to obtain the 
%   desired approximate solution. This region is defined as
%
%       lambda^2  = { 0       if smallest_singular_value ? eps
%                   { (1 - (sigma_eps/eps)^2)*lambda_max^2  otherwise
%
% [10]  S. Chiaverini, B. Siciliano, and O. Egeland, “Review of the damped
%       least-squares inverse kinematics with experiments on an industrial 
%       robot manipulator”, IEEE Trans. on Control Systems Technology, 
%       vol. 2, no. 2, pp. 123–134, June 1994.

if nargin < 5
	ref = 0;
    if nargin < 4
        lambda_max = 100;
        if nargin < 3
            eps = 1e-6;
        end
    end
end

mat = mat_from_struct(robj);

J_inv = mex_jacobian_DLS_inv(mat, q, eps, lambda_max, ref);