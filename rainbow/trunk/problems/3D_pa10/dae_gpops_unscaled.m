function [dae, derivdae] = dae_gpops_unscaled(sol)
% Compute the differential algebraic constraints for a PA-10 robotic
% manipulator:
%
%   f1 = x1_dot = x2;
%   f2 = x2_dot = inv(M(x1)) * (u - N(x1,x2));
%
% where,
%
%   x1 = [q1,...,q6]';
%   x2 = [q1_dot,...,q6_dot]';
%

% Pull out variables
t = sol.time;
x1 = sol.state(:,1:6);
x2 = sol.state(:,7:12);
u = sol.control;
N = length(t);  % Number of nodes

f1 = x2;
[x2_dot, df2_dq, df2_dqp, df2_dtau] = pa10_acceleration(u',x1',x2');
f2 = x2_dot';
dae = [f1, f2];

% Compute derivative
df1_dx1 = zeros(N*6,6);
df1_dx2 = blkdiag(ones(N,1),ones(N,1),ones(N,1),ones(N,1),ones(N,1),ones(N,1));
df1_du = zeros(N*6,6);
df1_dt = zeros(N*6,1);

df2_dx1 = zeros(size(df1_dx1));
df2_dx2 = zeros(size(df1_dx2));
df2_du = zeros(size(df1_du));
df2_dt = df1_dt;

j = 1;
for n = 1:6
    for k = 1:N
        df2_dx1(j,:) = df2_dq(n,:,k);
        df2_dx2(j,:) = df2_dqp(n,:,k);
        df2_du(j,:) = df2_dtau(n,:,k);
        j = j+1;
    end
end

derivdae = [df1_dx1, df1_dx2, df1_du, df1_dt; ...
            df2_dx1, df2_dx2, df2_du, df2_dt];
