function tinv = inv_tmatrix(t)
tinv = eye(4);
tinv(1:3,1:3) = t(1:3,1:3)';
tinv(1:3,4) = -1 * tinv(1:3,1:3) * t(1:3,4);