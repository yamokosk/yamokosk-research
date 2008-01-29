function [M, b, g] = dynfun_2Drobots(q, qd, udata)
Q = reshape(q, 3, 2);
Qd = reshape(qd, 3, 2);

M1 = inertia(udata.r1, Q(:,1));
M2 = inertia(udata.r2, Q(:,2));
M = [M1, zeros(3,3); zeros(3,3), M2];

C1 = coriolis(udata.r1, Q(:,1)', Qd(:,1)');
C2 = coriolis(udata.r2, Q(:,2)', Qd(:,2)');
b = [C1'; C2'];

G1 = gravload(udata.r1,Q(:,1)');
G2 = gravload(udata.r2,Q(:,2)');
g = [G1'; G2'];