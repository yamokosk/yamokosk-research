function [dae, derivdae] = dae_gpops(sol)
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
global CONSTANTS;

% Pull out variables
bounds = CONSTANTS.bounds;
tau = sol.time;
y1 = sol.state(:,1:6);
y2 = sol.state(:,7:12);
mu = sol.control;
N = length(tau);  % Number of nodes

f1 = y2;

% Descale quantities so 
u = zeros(size(mu));
x1 = zeros(size(y1));
x2 = zeros(size(y2));
for n = 1:N
    u(n,:) = descale(mu(n,:), bounds.u_lb', bounds.u_ub');
    x1(n,:) = descale(y1(n,:), bounds.x_lb(1:6,1)', bounds.x_ub(1:6,1)');
    x2(n,:) = descale(y2(n,:), bounds.x_lb(1:6,1)', bounds.x_ub(1:6,1)');
end

[x2_dot, df2_dx1, df2_dx2, df2_du] = pa10_acceleration(u',x1',x2');
x2_dot = x2_dot';

% Scale accelerations
f2 = zeros(size(x2_dot));
for n = 1:N
    f2(n,:) = scale(x2_dot(n,:), bounds.x2_lb', bounds.x2_ub');
end

dae = [f1, f2];

% Compute derivative
df1_dy1 = zeros(N*6,6);
df1_dy2 = blkdiag(ones(N,1),ones(N,1),ones(N,1),ones(N,1),ones(N,1),ones(N,1));
df1_dmu = zeros(N*6,6);
df1_dtau = zeros(N*6,1);

df2_dy1 = zeros(size(df1_dy1));
df2_dy2 = zeros(size(df1_dy2));
df2_dmu = zeros(size(df1_dmu));
df2_dtau = df1_dtau;

%[junk, df2dy1, df2dy2, df2dmu] = pa10_acceleration(mu',y1',y2');

j = 1;
for n = 1:6
    for k = 1:N

        scale_dfdx1 = (bounds.x_ub(1:6) - bounds.x_lb(1:6))/(bounds.x2_ub(n) - bounds.x2_lb(n));
        %scale_dfdx2 = (bounds.x_ub(7:12) - bounds.x_lb(7:12))/(bounds.x2_ub(n) - bounds.x2_lb(n));
        scale_dfdx2 = ones(6,1) / (bounds.t_ub-bounds.t_lb);
        scale_dfdu = (bounds.u_ub(1:6) - bounds.u_lb(1:6))/(bounds.x2_ub(n) - bounds.x2_lb(n));
        df2_dy1(j,:) = df2_dx1(n,:,k) .* scale_dfdx1';
        df2_dy2(j,:) = df2_dx2(n,:,k) .* scale_dfdx2';
        df2_dmu(j,:) = df2_du(n,:,k) .* scale_dfdu';
        %         df2_dy1(j,:) = scale(df2dy1(n,:,k), ;
        %         df2_dy2(j,:) = df2dy2(n,:,k);
        %         df2_dmu(j,:) = df2dmu(n,:,k);

        j = j+1;
    end
end

derivdae = [df1_dy1, df1_dy2, df1_dmu, df1_dtau; ...
            df2_dy1, df2_dy2, df2_dmu, df2_dtau];
