function [pt,t] = ray_sphere_int(P0,Pd,Sc,Sr)
A = Pd' * Pd;
B = 2 * Pd' * (P0 - Sc);
C = (P0 - Sc)' * (P0 - Sc) - Sr^2;

discriminant = B^2 - 4*A*C;
if discriminant >= 0
    pt = zeros(3,2);
    
    t0 = ( -B - sqrt(discriminant) )/(2*A);
    t1 = ( -B + sqrt(discriminant) )/(2*A);
    
    pt(:,1) = P0 + t0 * Pd;
    pt(:,2) = P0 + t1 * Pd;
    t = [t0, t1];    
else
    pt = [];
    t = [];
end
