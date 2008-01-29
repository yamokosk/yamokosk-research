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
robothandles = [];
psi_old = [];
t_old = [];
[dim, npts] = size(tau);
tvec = linspace(tspan(1), tspan(2), npts);

options = odeset('OutputFcn',@odeprint);
[tout, xout] = ode23s(@odefun, tspan, [q_init; qp_init], options);

    function x_dot = odefun(t, x)
        x_dot = zeros(size(x));
        X = reshape(x, dim, 2);
        [M, b, g] = dynfun(X(:,1), X(:,2), userdata);
        [psi, psi_dot] = derivapprox(confun, X(:,1), X(:,2), userdata);
        tau_i = interp1(tvec, tau', t)';
        
        M_inv = inv(M);
        q_star_dbl_dot = M_inv * (tau_i - b - g);
        psi_bar = M_inv * psi' * pinv( psi * M_inv * psi' );
        q_con_dbl_dot = -psi_bar * psi_dot * X(:,2);
        q_dbl_dot = q_con_dbl_dot + (eye(6) - psi_bar * psi) * q_star_dbl_dot;
        
        x_dot = [X(:,2); q_dbl_dot];
    end

    function status = myplot(t,x,flag)
        switch (flag)
            case 'init'
                figure(1);
                robothandles.r1 = plot(userdata.r1, x(1:3,1)');
                robothandles.r2 = plot(userdata.r2, x(4:6,1)');
                
                phi = confun(x(1:6), userdata);
                figure(2);
                hold on;
                plot(t(1), phi(1), 'x');
                %plot(t(1), phi(2), 'o');
                %solutions = x;
            case 'done'
%                 h1 = userdata.r1; h2 = userdata.r2;
%                 for n = 1:numcols(solutions)
%                     h1 = plot(h1,solutions(1:3,n)');
%                     h2 = plot(h2,solutions(4:6,n)');
%                 end
            otherwise
                %solutions = [solutions, x];
                for ti = 1:length(t)
                    figure(1);
                    robothandles.r1 = plot(robothandles.r1, x(1:3,ti)');
                    robothandles.r2 = plot(robothandles.r2, x(4:6,ti)');
                    figure(2);
                    phi = confun(x(1:6,ti), userdata);
                    plot(t(ti), phi(1), 'x');
                    %plot(t(ti), phi(2), 'o');
%                     psi_old = psi;
%                     [psi, psi_dot] = derivapprox(confun, X(:,1), X(:,2), userdata);
%                     t_old = t;
                end
        end
        status = 0;
    end

xrow = xout(end,:)';
Xout = reshape(xrow, dim, 2);
q = Xout(:,1);
qp = Xout(:,2);

end


function [psi, psi_dot] = derivapprox(f,q,q_dot,udata)
opts.userdata = udata;
[psi, H] = numderiv(f, q, opts);
psi_dot = zeros(size(psi));
for c = 1:numcols(psi_dot)
    psi_dot(:,c) = squeeze(H(:,:,c))*q_dot;
end
end