% RLG - Random Loop Generator
function [states, elapsed] = rlg(scene,rsrc,rsen,target)
% target struct
%   target.del - Max cartesian error in the target X-Y plane
%   target.dbar - Mean distance between sensor and tracked object
%   target.sigma_d - Allowable std in distance between tracked object
%   and sensor
%   target.Lbar - Mean distance between sensor and source reference frames
%   target.sigma_L - Allowable std in Lbar
%   target.sigma_theta - Allowable std in view orientation error
%   target.gvars - Gait parameters

debug = false;
q_names = {'src1';'src2';'src3';'src4';'src5';'src6';'sen1';'sen2';'sen3';'sen4';'sen5';'sen6'};
scene = buildScene(scene, [ [q_names, num2cell(zeros(12,1)); target.gvars] ]);
targetBody = getBody(scene,'target');
T_wcs_d = targetBody.T_wcs_obj;
states = [];

if debug
    h = figure;
    hold on; grid on;
    drawCoordinateSystem(h, eye(4), 'wcs');
    drawCoordinateSystem(h, T_wcs_d,'d');
    drawCoordinateSystem(h, rsen.base, 'rsen.base');
    drawCoordinateSystem(h, rsrc.base, 'rsrc.base');
end

tic;
% -------------------------------------------------------------------------
% Compute T_wcs_sen
%
% T_d_dp - Cartesian tracking error in the target X-Y plane
%   target.del defines the radius for a circle in the X-Y plane of
%   T_wcs_d in which the principle vector of the imaging system must pass
%   through.
alpha = rand * 2 * pi - pi; % Random rotation about target z-axis
T_d_dp = transl([target.del*cos(alpha); target.del*sin(alpha); 0]);

% T_dp_dhat - View orientation error
%   theta is the angle measure between the desired viewing vector and the 
%   actual one. target.sigma_theta allows the user to define an allowable
%   standard deviation in this parameter.
phi = rand * 2 * pi - pi; % Random rotation of the local X-Z plane
T_dp_dpp = rotz(phi);
theta = target.sigma_theta*randn; % View orientation error
T_dpp_dhat = roty(theta);
T_dp_dhat = T_dp_dpp * T_dpp_dhat;

% T_dhat_sen - Translate [0; 0; -dbar +/- signma_d]
% d = target.dbar + target.sigma_d*randn; % distance between sen and target
% T_dhat_sen = transl([0; 0; -d]);
% Different approach. Shoot a ray aligned with the z-axis of T_wcs_dhat and
% calculate where (and if) it intersects the reachable workspace of the
% robots
T_wcs_dhat = T_wcs_d * T_d_dp * T_dp_dhat;
P_wcs_dhat0 = T_wcs_dhat(1:3,4);
P_wcs_dhat_mZ = P_wcs_dhat0 - T_wcs_dhat(1:3,3);
P_wcs_dhat_pZ = P_wcs_dhat0 + T_wcs_dhat(1:3,3);

T_wcs_rsen_S1 = fkine(rsen,0);
P_wcs_rsen_S1 = T_wcs_rsen_S1(1:3,4);

T_wcs_rsrc_S1 = fkine(rsrc,0);
P_wcs_rsrc_S1 = T_wcs_rsrc_S1(1:3,4);

[senInt,senT] = ray_sphere_int(P_wcs_dhat0, P_wcs_dhat_mZ, P_wcs_rsen_S1, .86);
[srcInt,srcT] = ray_sphere_int(P_wcs_dhat0, P_wcs_dhat_pZ, P_wcs_rsrc_S1, .86);

if isempty(senT) || isempty(srcT)
    elapsed = toc;
    return; % No intersections.. no need to process further
else
    % Deal with sen then src
    d = computeDist(target.dbar, target.sigma_d, senT(1));
    L = computeDist(target.Lbar-d, target.sigma_L, srcT(1));
end

% T_wcs_sen - Sensor frame in the WCS
T_dhat_sen = transl([0; 0; -d]);
T_wcs_sen = T_wcs_dhat * T_dhat_sen;
if debug
    drawCoordinateSystem(h,T_wcs_sen, 'sen');
end

% -------------------------------------------------------------------------
% Compute T_wcs_src
%
% T_wcs_src - Source frame in the WCS
T_dhat_src = transl([0; 0; L]) * rotz(phi);
T_wcs_src = T_wcs_dhat * T_dhat_src;

% -------------------------------------------------------------------------
% Finally compute the robot joint angles
q_rsrc = ika(rsrc,T_wcs_src);
q_rsen = ika(rsen,T_wcs_sen);
elapsed = toc;

ptr = 1;
for n = 1:size(q_rsrc,1)
    for m = 1:size(q_rsen,1)
        q_combined = [q_rsrc(n,:)'; q_rsen(m,:)'];
        q = [q_names, num2cell(q_combined)];
        scene = buildScene(scene, [q; target.gvars]);
        
        if ( ~dSimpleSpaceCollide2(scene.space{1}.id,scene.space{2}.id) && ... % Environment with source 
             ~dSimpleSpaceCollide2(scene.space{1}.id,scene.space{3}.id) && ... % Environment with sensor
             ~dSimpleSpaceCollide2(scene.space{2}.id,scene.space{3}.id) && ... % source with sensor
             ~dSimpleSpaceCollide2(scene.space{2}.id,scene.space{4}.id) && ... % source with human
             ~dSimpleSpaceCollide2(scene.space{3}.id,scene.space{4}.id) ) % sensor with human
            states(ptr,:) = [q_rsrc(n,:), q_rsen(m,:)];
            ptr = ptr + 1;
        end
    end
end
end

function d = computeDist(dbar, sigma, t0)
if t0 < 0
    % P_wcs_dhat0 is inside of the sensor reach... process as usual.
    d = dbar + sigma*randn; % distance between sen and target
    if d < t0 d = t0; end
else
    if dbar < t0
        d = t0;
    else
        d = rand(1)*(dbar-t0) + t0;
    end
end
end

function int = intersect_lineline(x1,x2,x3,x4)
% x1 and x2 are on line1
% x3 and x4 are on line2
a = x2 - x1;
b = x4 - x3;
c = x3 - x1;
s = dot(cross(c,b), cross(a,b))/norm(cross(a,b))^2;
int = x1 + a*s;
end

function int = intersect_lineplane(x1,x2,x3,x4,x5)
A = [1, 1, 1, 1; x1, x2, x3, x4];
B = [1, 1, 1, 0; x1, x2, x3, x5-x4];
t = -det(A)/det(B);
int = x4 + (x5 - x4)*t;
end
% 
% % Step 1. Determine x-y bounds and choose (xr,yr) within those bounds
% cen1 = transl( R1.base );
% Rext1 = norm( transl( fkine(R1, zeros(1,6)) ) - transl( fkine(R1, 0) ) );
% 
% cen2 = transl( R2.base );
% Rext2 = norm( transl( fkine(R2, zeros(1,6)) ) - transl( fkine(R2, 0) ) );
% 
% [num_int, p] = circles_imp_int_2d(Rext1, cen1(1:2,1), Rext2, cen2(1:2,1));
% 
% if ( (num_int < 2) || (num_int == 3) )
%     error('Too many or too little intersections of RWS');
% end
% 
% d = Rext1 - p(1,1);
% xbnd = [p(1,1) - d, p(1,1) + d];
% ybnd = [min(p(2,:)), max(p(2,:))];
% 
% bFoundAngBounds = false; q1 = []; q2 = []; b = [];
% junk2 = 0;
% while (~bFoundAngBounds)
% 
%     bFoundZBounds = false; zr = 0;
%     while (~bFoundZBounds)
%         xr = xbnd(1) + (xbnd(2)-xbnd(1)).*rand(1,1);
%         yr = ybnd(1) + (ybnd(2)-ybnd(1)).*rand(1,1);
% 
%         % Step 2. Determine bounds on z by checking intersections of a line
%         % perpendicular to x-y plane and through (xr,yr). If enough
%         % intersections exist, choose zr. Otherwise, re-pick (xr,yr) and try
%         % again.
%         s1 = [cen1',Rext1];
%         s2 = [cen2',Rext2];
%         lineTest = createLine3d([xr,yr,0],[xr,yr,1]);
% 
%         p1 = intersectLineSphere(lineTest,s1);
%         p2 = intersectLineSphere(lineTest,s2);
% 
%         if ( (size(p1,1) > 1) || (size(p2,1) > 1) )
%             bFoundZBounds = true;
%             p = [p1; p2];
%             zbnd = [0, min( abs(p(:,3)) )];
% 
%             zr = zbnd(1) + (zbnd(2)-zbnd(1)).*rand(1,1);
%         end
%     end
%     b = [xr,yr,zr]';
%     %b1 = b - cen1;
%     %b2 = b - cen2;
% 
%     % Step 3. The hard part. For each rotation of the virtual body, arcs of
%     % the endpoints of the body are swept through space. The interesection
%     % of these arcs with the spheres determine the range for each body
%     % rotation.
%     %
%     % If the following is true for any sphere,
%     %
%     %       b + ||a|| * b/||b|| < Rext
%     %
%     % then any set of rotations are admissible. Likewise, if
%     %
%     %       b - ||a|| * b/||b|| > Rext
%     %
%     % then no rotations are admissible and a new (xr,yr,zr) needs to be
%     % generated. Otherwise, an interval for each rotation needs to be
%     %constructed by minimizing:
%     %
%     %       || a + b || - R
% 
%     T = eye(4);
% 
%     % Rotation sequence: 1-2-3 (X-Y-Z)
%     %  X-rotation range
%     n1 = T(1:3,2); n2 = T(1:3,3);
% 
%     [numPts, junk2, ang1] = intersectionCircleSphere3D(cen1,Rext1,b,n1,n2,d_source_min);
%     if numPts == 0,  continue; end
% 
%     [numPts, junk2, ang2] = intersectionCircleSphere3D(cen2,Rext2,b,n1,n2,d_detector_min);
%     if numPts == 0,  continue; end
% 
%     ang(1) = max(ang1(1),ang2(1));
%     ang(2) = min(ang1(2),ang2(2));
%     angr = ang(1) + (ang(2)-ang(1)).*rand(1,1);
%     T = T * rotx(angr);
% 
%     %  Y-rotation range
%     n1 = T(1:3,1); n2 = T(1:3,3);
% 
%     [numPts, junk2, ang1] = intersectionCircleSphere3D(cen1,Rext1,b,n1,n2,d_source_min);
%     if numPts == 0,  continue; end
% 
%     [numPts, junk2, ang2] = intersectionCircleSphere3D(cen2,Rext2,b,n1,n2,d_detector_min);
%     if numPts == 0,  continue; end
% 
%     ang(1) = max(ang1(1),ang2(1));
%     ang(2) = min(ang1(2),ang2(2));
%     angr = ang(1) + (ang(2)-ang(1)).*rand(1,1);
%     T = T * roty(angr);
% 
%     %  Z-rotation range
%     n1 = T(1:3,1); n2 = T(1:3,2);
% 
%     [numPts, junk2, ang1] = intersectionCircleSphere3D(cen1,Rext1,b,n1,n2,d_source_min);
%     if numPts == 0,  continue; end
% 
%     [numPts, junk2, ang2] = intersectionCircleSphere3D(cen2,Rext2,b,n1,n2,d_detector_min);
%     if numPts == 0,  continue; end
% 
%     ang(1) = max(ang1(1),ang2(1));
%     ang(2) = min(ang1(2),ang2(2));
%     angr = ang(1) + (ang(2)-ang(1)).*rand(1,1);
%     T = T * rotz(angr);
% 
%     T(1:3,4) = b;
% 
%     % Now get inverse kinematic solutions from robots
%     % R1
%     T1 = T;
%     endpt1 = T1 * [0,0,-d_source_min,1]';
%     T1(1:3,4) = endpt1(1:3,1);
%     q1 = ika(R1,T1);
% 
%     % R2
%     T2 = T * rotx(pi);
%     T2(1:3,4) = b;
%     endpt2 = T2 * [0,0,-d_detector_min,1]';
%     T2(1:3,4) = endpt2(1:3,1);
%     q2 = ika(R2,T2);
% 
%     if ( (size(q2,1) > 0) && (size(q1,1) > 0) )
%         bFoundAngBounds = true;
%     end
% end
% 
% 
% if debug
%     clf
%     %h = gcf;
%     gcf;
%     hold on
% 
%     plot(R1,q1(1,:), 'nobase');
%     plot(R2,q2(1,:), 'nobase');
% 
%     surfh1 = drawSphere(cen1,Rext1);
%     set(surfh1, 'FaceAlpha', 0.1, 'FaceColor', [0.2, 0.5, 0.2]);
% 
%     surfh2 = drawSphere(cen2,Rext2);
%     set(surfh2, 'FaceAlpha', 0.1, 'FaceColor', [0.5, 0.2, 0.5]);
% 
%     %         plane = createPlane([-1.5,0,0],[1.5+1.3,0,0],[1.3/2,0,1.5]);
%     %         surfh3 = drawPlane3d(plane);
%     pts = [-1.5, 0, 1.3170; -1.5, 0, 0; 1.5+1.3, 0, 0; 1.5+1.3, 0, 1.317];
%     surfh3 = patch(pts(:, 1), pts(:, 2), pts(:, 3), 'm');
%     set(surfh3, 'FaceAlpha', 0.1, 'FaceColor', [0,0,0]);
% 
%     % Draw virtual bodies
%     % [x1 y2 z1 x2 y2 z2 r]
%     cylh1 = drawCylinder([b',T1(1:3,4)',0.05]);
%     set(cylh1, 'FaceColor', [0.2, 0.5, 0.2]);
% 
%     cylh2 = drawCylinder([b',T2(1:3,4)',0.05]);
%     set(cylh2, 'FaceColor', [0.5, 0.2, 0.5]);
% 
%     axis([-1.5 (1.3+1.5) -1.5 1.5 0 1.5]);
% end