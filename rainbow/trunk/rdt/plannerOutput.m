function stop = plannerOutput(iter, status, path, message, flag, Prob)
% rdtOutput - Generic RDT output function
%
%   rdtOutput(Prob) creates 
%   a structure containing all the necessary information for the RDT
%   implementation in Matlab. The inputs are described below:
%
%   flag - 'init', [], 'done'

stop = false;
switch (flag)
    case 'init'
        fprintf('Iteration     Status     Comment\n');
        fprintf('---------     ------     -------\n');
%         figure(2); set(2,'Position',[50,50,560,420]); grid on; view(45,45);
%         figure(3); set(3,'Position',[620,50,560,420]); grid on; view(45,45);
    case 'done'
        fprintf('RDT complete.\n');
    case 'iter'
        fprintf('%s     %s    %s\n', iter, status, message);
    case 'connect'
        fprintf('%s       %s    %s\n', iter, status, message);
        % Print info about this iteration
        if (~isempty(path))
            figure(1); hold on;
            N = size(path,2);
            Xsrc = zeros(2,N); Qsrc = zeros(3,N);
            Xsen = zeros(2,N); Qsen = zeros(3,N);
            for n = 1:N
                Qsrc(:,n) = path(2:4,n);
                Tsrc = fkine_planar_pa10(Qsrc(:,n),Prob.userdata.r1);
                %plot_planar_pa10(qsrc, Prob.userdata.r1, gcf);
                Xsrc(:,n) = Tsrc(1:2,4);

                Qsen(:,n) = path(8:10,n);
                Tsen = fkine_planar_pa10(Qsen(:,n),Prob.userdata.r2);
                %plot_planar_pa10(qsen, Prob.userdata.r2, gcf);
                Xsen(:,n) = Tsen(1:2,4);
            end
            % THERE IS A PROBLEM IN THE RANDOM STATE GENERATOR.. THE VELOCITIES
            % ARE TOO HIGH AT TIMES.. THINK THIS IS CAUSING PROBLEMS.
            plot(Xsrc(1,:), Xsrc(2,:), 'k.-');
            plot(Xsen(1,:), Xsen(2,:), 'r.-');
            
%             figure(2); hold on;
%             plot3(Qsrc(1,:),Qsrc(2,:),Qsrc(3,:),'*-')
%             figure(3); hold on;
%             plot3(Qsen(1,:),Qsen(2,:),Qsen(3,:),'*-')
            drawnow;
        end
    otherwise
        error('Unrecognized flag');     
end