function [Xr,rind] = planar_rand_state_camera(Prob)
panelArea = 426.02;     % Active area of X-ray detector in cm^2
                        %   PAXScan 2520 - 17.9 x 23.8 = 426.02 cm^2
r_not = 5.38;           % Equivalent object radius in cm
                        %   Artificial Fem/Tib ~ 2.12in or 5.38cm
d_min = 40;             % Distance of detector from object
d_max = 60;             %   min/max calculated as: (average human reaction 
                        %       time) x (min/max safety factor) x
                        %       (max robot velocity)

% Compute some random camera parameters
d = rand()*(d_max-d_min) + d_min;
theta = rand()*0.06;
[fmin,fmax] = bound_focal_length(theta,d,panelArea,r_not);
f = rand()*(f_max-f_min) + f_min;

udata = Prob.userdata;
rindm = randint(100,1,[1,udata.traj.n],sum(50*clock));
test = 1; %binornd(1,.8);
names = {'q_src_2'; 'q_src_3'; 'q_src_5'; 'q_sen_2'; 'q_sen_3'; 'q_sen_5'};

for n = 1:100

    if (test)
        rind = rindm(n);

        % Step 0. Randomly select desired target point
        v = [udata.traj.vx(rind); udata.traj.vy(rind); 0];
        nx = v / norm(v);
        ny = cross([0;0;1], nx);
        T_wcs_d = [ nx(1), ny(1), 0, udata.traj.x(rind); ...
            nx(2), ny(2), 0, udata.traj.y(rind); ...
            nx(3), ny(3), 1,                  0; ...
            0,     0, 0,                  1];
        T_wcs_d = T_wcs_d * rotz(pi/2);
        
Xr = [];