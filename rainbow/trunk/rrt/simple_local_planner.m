function branch = simple_local_planner(No,Nf)

% Cost function 
J = phi(xf,tf) + int_t0_tf( L(x(t),u(t));

% Dynamic system
function [q,qp] = fwddyncon(tau, q_init, qp_init, tspan, dynfun, confun, userdata)
% Solves dynamics problems of the form:
%   q'' = M_inv(q) * ( tau - b(q,q') - g(q) );
%   
%   s.t.
%
%       phi(q) = 0
%
% Where phi(q) is a set of m holonomic constraints. Several functions must
% be specified:
%
%   [psi, psi_dot] = confun(q, userdata)
%   [M, b, g] = dynfun(q, qd, userdata)
%
[dim, npts] = size(tau);
tvec = linspace(tspan(1), tspan(2), npts);

[tout, xout] = ode45(@odefun, tspan, [q_init; qp_init], []);

    function x_dot = odefun(t, x)
        X = reshape(x, dim, 2);
        [M, b, g] = dynfun(X(:,1), X(:,2), userdata);
        [phi, phi_dot] = confun(X(:,1), userdata);
        tau_i = interp1(tvec, tau, t);
        
        M_inv = pinv(M);
        q_star_dbl_dot = M_inv * (tau_i - b - g);
        phi_bar = M_inv * phi' * pinv( phi * M_inv * phi' );
        q_dbl_dot = -phi_bar * phi_dot * X(:,2) + (1 - phi_bar * phi) * q_star_dbl_dot;
    end

xrow = x(end,:)';
Xout = reshape(xrow, dim, 2);
q = Xout(:,1);
qp = Xout(:,2);

end