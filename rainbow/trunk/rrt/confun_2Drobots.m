function phi = confun_2Drobots(q, udata)
Q = reshape(q, 3, 2);
T_L_1 = fkine(udata.r1, Q(:,1));
T_L_2 = fkine(udata.r2, Q(:,2));

T_2_1 = T_inv(T_L_2) * T_L_1;

phi = zeros(1,1);

phi(1) = 1 + dot(T_L_1(1:3,1), T_L_2(1:3,1));
%phi(2) = dot(T_2_1(1:3,4), [0;1;0]);

function Tinv = T_inv(T)
Tinv = eye(4);
Tinv(1:3,1:3) = T(1:3,1:3)';
Tinv(1:3,4) = -Tinv(1:3,1:3) * T(1:3,4);