function Z = map3DMetric(X,Y,zslice,target_mean,target_var,f,udata)
srcRbt = udata.rsrc;
senRbt = udata.rsen;
neval = @(x, target) f.neval(x, target, udata);

Z = zeros(size(X));
[N,N] = size(X);
starttime = now;
firstpass = true;

for r = 1:N
    for c = 1:N
        v = zeros(6,1);
        P_src = [X(r,c); Y(r,c); zslice];
        
        % Fixed orientation
%         k = [0; 1; 0] * -pi/2;
        
        % Orientation set to point towards target point
        P_target = target_mean(1:3,1);
        temp = P_target - P_src;
        z_axis = temp/norm(temp);
        x_axis = [0;0;1];           % x-axis of EE points in global +X
        y_axis = cross(z_axis,x_axis);
        y_axis = y_axis/norm(y_axis);
        T_EE = eye(4);
        T_EE(1:3,1:3) = [x_axis, y_axis, z_axis];
        k = irotk(T_EE);
        
        xr = [P_src; k; v; 1.3315; 0.5152]; % mean time and separation distance
        
        D = diag(target_var);
        Z(r,c) = nodeSensingEffectiveness(xr, target_mean, D, neval);
        
        if (firstpass)
            onepass = now;
            firstpass = false;
            
            diff = onepass - starttime;
            cyclesleft = N*N - 1;
            timeleft = fix(datevec((cyclesleft-1)*diff));
            finishtime = fix(datevec(starttime + cyclesleft*diff));
            fprintf(1,'Time left: %d days, %d hours, %d min, %d sec \n', timeleft(3), timeleft(4), timeleft(5), timeleft(6));
            fprintf(1,'Finish at: March %d, %d:%d:%d\n', finishtime(3), finishtime(4), finishtime(5), finishtime(6));
        end
    end
end