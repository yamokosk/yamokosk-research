function Z = map2DMetric(X,Y,zslice,target_mean,target_var,neval,udata)
srcRbt = udata.rsrc;
senRbt = udata.rsen;

Z = zeros(size(X));
[N,N] = size(X);

% Preliminaries
T_srcBase_EE = eye(4);
T_senBase_EE = eye(4);
yaxis = [0;0;1];

T_srcBase_EE(1:3,2) = yaxis;
T_senBase_EE(1:3,2) = yaxis;

d = 1.0;
P_target = [target(1:2); 0];
n_hat = [target(5:6); 0];

for r = 1:N
    for c = 1:N
        % Calculations for source robot
        P_srcEE = [X(r,c); Y(r,c); zslice];

        % METHOD 1: Point src at target
%         P_srcEE_target = P_target - P_srcEE;
%         U_srcEE_target = P_srcEE_target / norm(P_srcEE_target);

        % METHOD 2: Point src along global y-axis
        U_srcEE_target = [0; -1; 0];
        
        xtemp = cross(yaxis,U_srcEE_target);
        
        % Create source EE t-matrix
        T_srcBase_EE(1:3,1) = xtemp/norm(xtemp);
        T_srcBase_EE(1:3,3) = U_srcEE_target;
        T_srcBase_EE(1:3,4) = P_srcEE;
        
        % Calculations for sensor robot        
        P_senEE = P_srcEE + d*U_srcEE_target;
        U_senEE_target = -U_srcEE_target;
        xtemp = cross(yaxis,U_senEE_target);

        % Create sensor EE t-matrix
        T_senBase_EE(1:3,1) = xtemp/norm(xtemp);
        T_senBase_EE(1:3,3) = U_senEE_target;
        T_senBase_EE(1:3,4) = P_senEE;
        
        % Do inverse kinematics
        [Qsrc, srcSolnFound] = inv_kin(srcRbt, T_srcBase_EE);
        [Qsen, senSolnFound] = inv_kin(senRbt, T_senBase_EE);           

        % If solution is found, evaluate it
        if (srcSolnFound && senSolnFound)
            xr = [Qsrc(1:3,1); zeros(3,1); Qsen(1:3,1); zeros(3,1); target(end)];
            Z(r,c) = nodeSensingEffectiveness(x, target_mean, target_var, neval);
        end
    end
end