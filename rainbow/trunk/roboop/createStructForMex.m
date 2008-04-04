function toMex = createStructForMex(robj)

% A ndof × 23 matrix (kinematic, inertial and motor parameters) can also 
% be used. The structure of the initialization matrix is:
% Column Variable Description
% 1 ? joint type (revolute=0, prismatic=1)
% 2 ? Denavit-Hartenberg parameter
% 3 d Denavit-Hartenberg parameter
% 4 a Denavit-Hartenberg parameter
% 5 ? Denavit-Hartenberg parameter
% 6 ?min minimum value of joint variable
% 7 ?max maximum value of joint variable
% 8 ?off joint offset
% 9 m mass of the link
% 10 cx center of mass along axis x
% 11 cy center of mass along axis y
% 12 cz center of mass along axis z
% 13 Ixx element xx of the inertia tensor matrix
% 14 Ixy element xy of the inertia tensor matrix
% 15 Ixz element xz of the inertia tensor matrix
% 16 Iyy element yy of the inertia tensor matrix
% 17 Iyz element yz of the inertia tensor matrix
% 18 Izz element zz of the inertia tensor matrix
% 19 Im motor rotor inertia
% 20 Gr motor gear ratio
% 21 B motor viscous friction coefficient
% 22 Cf motor Coulomb friction coefficient
% 23 immobile flag for the kinematics and inverse kinematics
% (if true joint is locked, if false joint is free)
%
% I'm adding a 24th for the value of q if the joint is locked
        
mat = zeros(robj.dof,24);

c = struct2cell(robj.links);

mat(:,1:5) = cell2mat( c(1:5,:) )';
mat(:,6:8) = cell2mat( c(7:9,:) )';
mat(:,9) = cell2mat( c(12,:) )';
mat(:,10:12) = cell2mat( c(10,:) )';

I = reshape(cell2mat(c(17,:)), 3, 3, robj.dof);
mat(:,13) = squeeze(I(1,1,:));  % Ixx
mat(:,14) = squeeze(I(1,2,:));  % Ixy
mat(:,15) = squeeze(I(1,3,:));  % Ixz
mat(:,16) = squeeze(I(2,2,:));  % Iyy
mat(:,17) = squeeze(I(2,3,:));  % Iyz
mat(:,18) = squeeze(I(3,3,:));  % Izz

mat(:,19:22) = cell2mat( c(13:16,:) )';
mat(:,23) = cell2mat( c(18,:) )';
mat(:,24) = cell2mat( c(6,:) )';

toMex = struct( 'initrobot', mat, ...
                'T_f_base', robj.T_f_base, ...
                'T_EE_tool', robj.T_EE_tool );