function [ang1, ang2] = asin2(x)
ang1 = asin(x);
if (x > 0), ang2 = pi - ang1;
else ang2 = -pi - ang1; end