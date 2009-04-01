function Xf = final_point(t0,tf,x0,xf)

qp_max = [1, 1, 2, 2*pi, 2*pi, 2*pi]';

dt = tf-t0;
q0 = x0(1:6,1);
qf = xf(1:6,1);

gr = (1 + sqrt(5))/2 - 1;   % golden ratio
Xf = [];
for n = 0:10
    dq = gr^n * (qf - q0);
    v_mean = dq / dt;
    ind = find( abs(v_mean) > qp_max );
    
    if ( isempty( ind ) )
        Xf = x0 + gr^n * (xf - x0);
        break;
    end
end