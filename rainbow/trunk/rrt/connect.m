function [optnodes, N, fopt] = connect(y0,y1)

% Robot parameters
theta = [2; 1; 0.5; 0.25; 1.5; 1; 0.25; 0.2*zeros(6,1); 0.1*zeros(6,1); 0];

% Bspline setup
N = nmatrix(zeros(1,5), 3, 100);
nodes = zeros(5,6);
tvec = [];

% Optimization
options = optimset('lsqnonlin');
options = optimset(options, 'Display',          'iter', ...
        'MaxFunEvals',      4000 * 31, ...
        'MaxIter',          200);

% Start optimization
warning off all
optnodes = lsqnonlin(@cf, [0.1; reshape(nodes,5*6,1)], [], [], options);

    function f = cf(x)
       tf = x(1);
       nodes = reshape(x(2:end), 5, 6);
       tvec = linspace(0,tf,100)';
       
       % Do integration
       [t,y] = ode23(@odefun, [0, tf], y0);
       f = [tf; y1 - y(end,:)'];
    end

    function y_dot = odefun(t, y)
        tau = N * nodes;
        taui = interp1(tvec,tau,t)';
        y_dot = zeros(size(y));
        y_dot(1:6) = y(7:12);
        y_dot(7:12) = planar_robots_eom(taui, y(1:6,1), y_dot(1:6,1), theta);
    end

fopt = cf(optnodes);
warning on all
end