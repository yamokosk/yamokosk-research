function [Xr,rind] = planar_rand_state_obs(Prob)
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
        C1 = udata.r1.T_f_base(1:2,4); % - [0; udata.r1.l0];
        R1 = 1.0;
        [t1,t2] = ray_circle_int(P_wcs_dhat0(1:2), P_wcs_dhat_X1(1:2), C1, R1);
        if isempty(t1)
            qr = [];
            %fprintf('\trind: %d failed. No intersection with source\n', rind);
            continue;
        end

        % Robot #2
        P_wcs_dhat_X2 = P_wcs_dhat0 - T_wcs_dhat(1:3,1); % Point on the dhat, minus X-axis
        C2 = udata.r2.T_f_base(1:2,4); % + [0; udata.r2.l0];
        R2 = 1.0;
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
        %[Qsrc,converged] = ikine_planar_pa10(T_wcs_src, udata.r1);
        [Qsrc,converged] = inv_kin(udata.r1, T_wcs_src);
        if (~converged)
            qr = [];
            %fprintf('\trind: %d failed. No ikine solution for source\n', rind);
            continue;
        end

        %[Qsen,converged] = ikine_planar_pa10(T_wcs_sen, udata.r2);
        [Qsen,converged] = inv_kin(udata.r2, T_wcs_sen);
        if (~converged)
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

                qsrc = qr(2:4,1); qsen = qr(8:10,1);
                sceneSetVars(names, [qsrc; qsen]);
                if (~sceneCollisionState)
                    Xr(:,counter) = qr;
                    counter = counter + 1;
                end
            end
        end
        
        if (counter > 0)
            return;
        end

    else
        xr = Prob.x_range;
        xr(2) = 2*(pi/3);
        xr(8) = 2*(pi/3);
        lb = Prob.x_lb;
        lb(2) = -pi/3;
        lb(8) = -pi/3;
        Xr = xr .* rand(length(lb),1) + lb;
        
        qsrc = Xr(2:4,1); qsen = Xr(8:10,1);
        sceneSetVars(names, [qsrc; qsen]);
        if (~sceneCollisionState)
            return;
        end
    end
end

error('Could not generate a random state after 100 attempts');