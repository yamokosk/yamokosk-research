function [X,Y,Z,v] = map_metric_3d(target_mean,target_var,f,udata)
srcRbt = udata.rsrc;
senRbt = udata.rsen;
neval = @(x, target) f.neval(x, target, udata);

% Preliminaries
T_srcBase_EE = eye(4);
T_senBase_EE = eye(4);
yaxis = [0;0;1];

T_srcBase_EE(1:3,2) = yaxis;
T_senBase_EE(1:3,2) = yaxis;

% Create grid
Nx = 50;
%x = linspace(0,1,Nx);   % in meters
x = linspace(0,1000,Nx);   % in mm

Ny = 50;
%y = linspace(-.5,.5,Ny);    % in meters
y = linspace(-500,500,Ny);    % in mm

Nz = 50;
%z = linspace(0,2,Nz);  % in meters
z = linspace(0,1000,Nz);  % in mm

[X,Y,Z] = meshgrid(x,y,z);

% Evaluate points on the grid
h = waitbar(0,'Please wait...');
c = 1;
v = zeros(Nx,Ny,Nz);
D = diag(target_var);

% Fixed orientation
T = roty(-pi/2)*rotz(pi);
k = irotk(T);
xr = [zeros(3,1); k; zeros(6,1); 1.3315; 0.5152]; % mean time and separation distance

for i = 1:Nx
    for j = 1:Ny
        for k = 1:Nz
            %P_src = [X(i,j,k); Y(i,j,k); Z(i,j,k)];    % in meters
            P_src = [X(i,j,k); Y(i,j,k); Z(i,j,k)]/1000;    % in mm

            % Orientation set to point towards target point
            %         P_target = target_mean(1:3,1);
            %         temp = P_target - P_src;
            %         z_axis = temp/norm(temp);
            %         x_axis = [0;0;1];           % x-axis of EE points in global +X
            %         y_axis = cross(z_axis,x_axis);
            %         y_axis = y_axis/norm(y_axis);
            %         T_EE = eye(4);
            %         T_EE(1:3,1:3) = [x_axis, y_axis, z_axis];
            %         k = irotk(T_EE);

            xr(1:3,1) = P_src;

            v(i,j,k) = nodeSensingEffectiveness(xr, target_mean, D, neval);
            waitbar(c/(Nx*Ny*Nz));
            c = c + 1;
        end
    end
end

close(h);