function q = ikine_planar_pa10(T_w_t, T_w_0, T_3_t)

T_0_3 = inv_tmatrix(T_w_0) * T_w_t * inv_tmatrix(T_3_t);

x = T_0_3(1,4);
y = T_0_3(2,4);
c123 = T_0_3(1,1);
s123 = T_0_3(2,1);

l1 = 0.45;
l2 = 0.48;

% Map p -> q
% q2
c2 = (x^2 + y^2 - l1^2 -l2^2) / (2*l1*l2);
if ( abs(c2) > 1 )
    q = [];
    return;
end
s2 = sqrt(1 - c2^2);
q2 = [atan2(s2, c2); atan2(-s2, c2)];

% q1
k1 = l1 + l2*c2;
k2 = l2*s2;
q1 = [atan2(y,x) - atan2(k2,k1); atan2(y,x) - atan2(-k2,k1)]

% q3
phi = [atan2(s123, c123); atan2(s123, c123)];
q3 = phi - q1 + q2;

q = [q1, q2, q3];


function tinv = inv_tmatrix(t)
tinv = eye(4);
tinv(1:3,1:3) = t(1:3,1:3)';
tinv(1:3,4) = - tinv(1:3,1:3) * t(1:3,4);