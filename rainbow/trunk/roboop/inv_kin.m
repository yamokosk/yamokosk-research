function [Q, converged] = inv_kin(robj, T_obj, mj, endlink);
%   Syntax
%       
%       [Q, converged] = inv_kin(robj, T_obj);
%       [Q, converged] = inv_kin(robj, T_obj, mj);
%       [Q, converged] = inv_kin(robj, T_obj, mj, endlink);
%
%   Description
% 
%   The inverse kinematic model is computed using a Newton-Raphson 
%   technique. If mj == 0, it is based on the following [1]:
%   
%       Tn(q*) = Tn(q + del_q) ~ Tn(q)*del_T(del_q) = T_obj
%       del_T(del_q) = inv(Tn(q)) * T_obj = I + delta
%                [     0, -del_z,  del_y, dx]
%       delta  = [ del_z,      0, -del_x, dy]
%                [-del_y,   delx,      0, dz]
%                [0, 0, 0, 0]
%       del_X  = [dx dy dz del_x del_y del_z]'
%       del_X  ~ J(q) * del_q
% 
%   If mj == 1, it is based on the following Taylor expansion [1,2]:
%
%       Tn(q*) = Tn(q + del_q) ~ Tn(q) + sum_1_n( (dTn/dqi) * del_qi) )
%
%   The function dTdqi computes these partial derivatives.
%
%   Given the desired position represented by the homogeneous transform
%   Tobj, this function return the column vector of joint variables that is corresponding
%   to this position. On return, the value converge is true if the
%   procedure has converge to values that give the correct position and false
%   otherwise.
%
%   Note: mj == 0 is faster (? 1.8×) than mj == 1. Also, mj == 1 might
%   converge when mj == 0 does not.
%
%   [1] B. Gorla and M. Renaud, Mod`eles des robots manipulateurs, 
%       application `a leur commande, Cepadues-´editions, Toulouse, mai 
%       1984.
%   [2] M. Lillholm E.B. Dam, M. Koch and, “Quaternions, interpolation and
%       animation”, Tech. Rep. DIKU-TR-98/5, University of Copenhagen,
%       July 1998.

if nargin < 4
    endlink = robj.dof;
    if nargin < 3
        mj = 0;
    end
end

mat = mat_from_struct(robj);

[Q, converged] = mex_inv_kin(mat, T_obj, mj, endlink);