function J = jacobian(robj, q, endlink, ref)
%   Syntax
%       
%       J = jacobian(robj, q, endlink, ref)
%
%   Description
%
%   The manipulator Jacobian defines the relation between the velocities 
%   in joint space q? and in the Cartesian space X expressed in frame i:
%
%       Xi = Ji(q) * q_dot
%
%   or the relation between small variations in joint space del_q and 
%   small displacements in the Cartesian space del_X:
%
%       del_Xi ~ Ji(q) * del_q
%
%   The manipulation Jacobian expressed in the base frame is given by 
%   (see [1]):
% 
%       J0(q) = [J0_1(q), J0_2(q), ..., J0_n(q)]
%
%   with
%
%       J0_i(q) = [ cross(z_i-1, p_i-1_n) ]   for a revolute joint
%                 [         z_i-1         ]
% 
%       J0_i(q) = [z_i-1]  for a prismatic joint
%                 [  0  ]
%
%   where z_i-1 and p_i-1_n are expressed in the base frame. Expressed in
%   the ith frame, the Jacobian is given by
%
%       Ji(q) = [R0_i',     0] * J0(q)
%               [    0, R0_i']
%
%   This function returns Ji(q) (i = 0 when not specified) for the endlink
%   (last link when not specified).

if nargin < 4
	ref = 0;
    if nargin < 3
        endlink = robj.dof;
    end
end

toMex = createStructForMex(robj);

J = mex_jacobian(toMex, q, endlink, ref);