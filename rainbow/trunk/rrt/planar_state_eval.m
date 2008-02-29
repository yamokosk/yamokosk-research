function fitness = planar_state_eval(x, Prob)
udata = Prob.userdata;
N = size(x,2);
fitness = zeros(N,1);

for n = 1:N
    % Unwrap the state vector
    t = x(1,n);
    qsrc = x(2:4,n);
    qpsrc = x(5:7,n);
    qsen = x(8:10,n);
    qpsen = x(11:13,n);

    % Get target's current position
    xt = interp1(udata.traj.t, udata.traj.x, t);
    yt = interp1(udata.traj.t, udata.traj.y, t);
    P_t = [xt; yt];
    vxt = interp1(udata.traj.t, udata.traj.vx, t);
    vyt = interp1(udata.traj.t, udata.traj.vy, t);
    v_t = [vxt; vyt];
    v_t_max = [max(abs(udata.traj.vx)); max(abs(udata.traj.vy))];

    % Calculate relative velocity between target and sensor EE
    J_sen = jacobian_planar_pa10(qsen, udata.r2);
    v_senEE = J_sen * qpsen;
    v_rel = v_t - v_senEE;

    % Calculate distance between target and a line formed by a line segment
    % between the EE of both robots.
    T_senEE = fkine_planar_pa10(qsen, udata.r2);
    T_srcEE = fkine_planar_pa10(qsrc, udata.r1);
    d = dist_Point_to_Segment(P_t, T_senEE(1:2,4), T_srcEE(1:2,4));

    % Calculate angle between X-axes of src and sen robots
    theta = abs(acos( dot(T_senEE(1:3,1), T_srcEE(1:3,1)) )) - pi;

    % Calculate projection of source EE to Y-Z plane of sensor EE
    Pw_srcEE = T_srcEE(:,4);
    Psen_srcEE = inv_tmatrix(T_senEE) * Pw_srcEE;
    proj = Psen_srcEE(2);

    % Finally the overall fitness of this state
    v_pen = norm(v_rel)/norm(v_t_max);
    off_pen = (d + proj)/0.2;
    ang_pen = theta/pi;
    fitness(n,1) = sqrt(v_pen^2 + off_pen^2 + ang_pen^2);
end


% dist_Point_to_Segment(): get the distance of a point to a segment.
%    Input:  a Point P and a Segment S (in any dimension)
%    Return: the shortest distance from P to S
function d = dist_Point_to_Segment(Pt, P0, P1)
v = P1 - P0;
w = Pt - P0;

c1 = dot(w,v);
if ( c1 <= 0 )
    d = norm(Pt - P0);
    return;
end

c2 = dot(v,v);
if ( c2 <= c1 )
    d = norm(Pt - P1);
    return;
end

b = c1 / c2;

Pb = P0 + b * v;
d = norm(Pt - Pb);