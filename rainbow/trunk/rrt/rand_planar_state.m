function xr = rand_planar_state(Prob)
% Random state generator for planar robot environment
udata = Prob.userdata;

% Two possibilities
%   1. Generate a completely random task space point.
%   2. Pick a task state from a normal distribution with mean equal to final
%   state.
% Then compute the associated robotic states using inverse kinematics.
test = binornd(1,udata.pi);
if test == 1
    % Pick uniformly random task space state
    xr = (Prob.x_ub - Prob.x_lb) .* rand(Prob.ns,1) + Prob.x_lb;
else
    % Pick random task space state from a normal distribtion about goal point
    xr = Prob.xf + udata.sf .* randn(Prob.ns,1);
end