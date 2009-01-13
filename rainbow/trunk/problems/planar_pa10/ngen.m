function xr = ngen(target, udata)

% Generate random deviant using the target's prefered viewing angle
sigma_theta = (45*pi/180)/6;
del_theta = sigma_theta*randn(1,1);

du = rotz(del_theta) * [target(3:4,:);0;1];
du = du(1:2,:);

% Generate random distance
sigma_dist = 1/6;
del_dist = 1 + sigma_dist*rand(1,1);

% Generate deviant
xr = [target(1:2,:) + del_dist*du; target(5)];
