function [Xr,rind] = planar_rand_state(Prob)
udata = Prob.userdata;
rindm = randint(100,1,[1,udata.traj.n],sum(50*clock));
% test = binornd(1,.8);
% 
% if (test)

    for n = 1:100
        %test = binornd(1,0.5);
        rind = rindm(n);
        %fprintf('Trying new rind: %d\n', rind);
        % if test > 0.5
        %     % Generate completely random joint configuration
        %     q1 = (udata.r1.qmax - udata.r1.qmin).*rand(1,3) + udata.r1.qmin;
        %     q2 = (udata.r2.qmax - udata.r2.qmin).*rand(1,3) + udata.r2.qmin;
        %
        %     % Now generate random task space velocity vector in the left to right
        %     % direction
        %     theta = (pi/(3*3)) * randn;
        %     speed = 0.8*rand;
        %     v = [speed*cos(theta); speed*sin(theta)];
        %
        %     % Get the corresponding joint speeds
        %     [Jsrc, Jinv_src] = jacobian_planar_pa10(q1, udata.r1);
        %     qp1 = Jinv_src * v;
        %
        %     [Jsen, Jinv_sen] = jacobian_planar_pa10(q2, udata.r2);
        %     qp2 = Jinv_sen * v;
        %
        %     % Report the new point
        %     qr = [udata.traj.t(rind); q1'; qp1; q2'; qp2];
        % else
        % Random state generator for planar robot environment

        % Step 0. Randomly select desired target point
        v = [udata.traj.vx(rind); udata.traj.vy(rind); 0];
        nx = v / norm(v);
        ny = cross([0;0;1], nx);
        T_wcs_d = [ nx(1), ny(1), 0, udata.traj.x(rind); ...
            nx(2), ny(2), 0, udata.traj.y(rind); ...
            nx(3), ny(3), 1,                  0; ...
            0,     0, 0,                  1];
        T_wcs_d = T_wcs_d * rotz(pi/2);
        %T_wcs_d = rotz(pi/8) * rotz(pi/2);
        %drawCoordinateSystem(gcf,T_wcs_d,num2str(rind));

        % Step 1. Create perturbation of the desired transformation matrix
        del = 20/1000; sigma_theta = (30*pi/180)/3;
        alpha = rand() * 2 * pi - pi;
        T_wcs_dhat = T_wcs_d * transl(del*cos(alpha), del*sin(alpha), 0) * rotz(sigma_theta * randn());

        % Step 2. Shoot a ray aligned with the z-axis of T_wcs_dhat and
        % calculate where (and if) it intersects the reachable workspace of the robots.
        % If the ray fails to intersect both robot workspaces, return false.
        t1=0.0; t2=0.0; t3=0.0; t4=0.0;
        P_wcs_dhat0 = T_wcs_dhat(1:3,4);

        % Robot #1
        P_wcs_dhat_X1 = P_wcs_dhat0 + T_wcs_dhat(1:3,1); % Point on the dhat, plus X-axis
        C1 = udata.r1.T_0(1:2,4); % - [0; udata.r1.l0];
        R1 = udata.r1.l1 + udata.r1.l2 + udata.r1.l3;
        [t1,t2] = ray_circle_int(P_wcs_dhat0(1:2), P_wcs_dhat_X1(1:2), C1, R1);
        if isempty(t1)
            qr = [];
            %fprintf('\trind: %d failed. No intersection with source\n', rind);
            continue;
        end

        % Robot #2
        P_wcs_dhat_X2 = P_wcs_dhat0 - T_wcs_dhat(1:3,1); % Point on the dhat, minus X-axis
        C2 = udata.r2.T_0(1:2,4); % + [0; udata.r2.l0];
        R2 = udata.r2.l1 + udata.r2.l2 + udata.r2.l3;
        [t3,t4] = ray_circle_int(P_wcs_dhat0(1:2), P_wcs_dhat_X2(1:2), C2, R2);
        if isempty(t3)
            qr = [];
            %fprintf('\trind: %d failed. No intersection with sensor\n', rind);
            continue;
        end

        % Deal with sen then src
        dbar = 0.1;
        sigma_d = 0.05/3;
        d = dbar + sigma_d * randn();

        % Clamp d so that point lies within the sensor workspace
        if (d < t3) d = t3; end
        if (d > t4) d = t4; end
        T_wcs_sen = T_wcs_dhat * transl(-d, 0, 0); % Sensor frame in the WCS

        % Do source robot
        Lbar = .3;
        sigma_L = 0.1/3;
        L = Lbar + sigma_L * randn();

        % Clamp L so that point lies within the source workspace
        if ( L < t1 ) L = t1; end
        if ( L > t2 ) L = t2; end
        T_wcs_src = T_wcs_dhat * transl(L, 0, 0) * rotz(pi); % Source frame in the WCS

        % Step 3. Use inverse kinematics to compute the robot joint angles
        Qsrc = ikine_planar_pa10(T_wcs_src, udata.r1);
        if (isempty(Qsrc))
            qr = [];
            %fprintf('\trind: %d failed. No ikine solution for source\n', rind);
            continue;
        end

        Qsen = ikine_planar_pa10(T_wcs_sen, udata.r2);
        if (isempty(Qsen))
            qr = [];
            %fprintf('\trind: %d failed. No ikine solution for sensor\n', rind);
            continue;
        end

        % Step 4. Count results
        nsrc = size(Qsrc,1);
        nsen = size(Qsen,1);
        ncol = nsrc * nsen;
        Xr = zeros(12+1,ncol); % 12 states + 1 for time
        counter = 1;

        % Step 5. Generate a perturbed task space velocity
        V_desired = repmat([udata.traj.vx(rind); udata.traj.vy(rind)], 1, ncol);
        V_task = V_desired + 0.1/3.*randn(2, ncol);

        % Step 6. Put results into single column style matrix
        for src = 1:nsrc
            [Jsrc,Jinv_src] = jacobian_planar_pa10(Qsrc(src,:), udata.r1);
            for sen = 1:nsen
                qr(1,1) = udata.traj.t(rind);
                qr(2:4,1) = Qsrc(src,:)';
                qr(5:7,1) = Jinv_src * V_task(:,counter);

                [Jsen,Jinv_sen] = jacobian_planar_pa10(Qsen(sen,:), udata.r2);
                qr(8:10,1) = Qsen(sen,:)';
                qr(11:13,1) = Jinv_sen * V_task(:,counter);

                Xr(:,counter) = qr;

                counter = counter + 1;
            end
        end

        return;
    end

    error('Could not generate a random state after 100 attempts');

% else
%     Xr = Prob.x_range .* rand(length(Prob.x_ub),1) + Prob.x_lb;
% end