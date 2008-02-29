function [Ni,We] = planar_local_planner(Ne, Nr, Prob)
% Connect algorithm - finds time optimal path and consideres joint angle
% and torque limits
t0 = Ne(1); x0 = Ne(2:end);
tf = Nr(1); xf = Nr(2:end);
N = 10;

try
    [t,U,X] = connect_min_effort(t0, x0, tf, xf, N, Prob.odefun, @planar_cost, Prob.userdata);
catch
     disp('Error in connect algorithm');
     %fprintf('t0: %f, tf: %f\n', t0, tf);
     %disp('[x0, xf]');
     %[x0, xf]
     %s = lasterror;
     %disp('Error message: ');
     %disp(s.message);
     Ni = [];
     We = [];
     return;    
end

Ni = [t'; X'];
trange = 1 ./ (2*[Prob.userdata.r1.umax, Prob.userdata.r2.umax]);
Trange = repmat(trange, N + 2, 1);
Unorm = ( U .* Trange );
We = sqrt( sum( Unorm .^ 2, 2 ) )';

% PROBLEM
% Control at end points can be different .. 