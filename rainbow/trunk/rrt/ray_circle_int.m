function [t1, t2] = ray_circle_int(P, D, C, R)

delta = P - C;
del = (dot(D,delta))^2 - norm(D)^2 * (norm(delta)^2 - R^2);

if (del < 0)
    t1 = []; t2 = [];
else
    t = zeros(2,1);
    t2 = ( -dot(D, delta) + sqrt(del) ) / (norm(D)^2);
    t1 = ( -dot(D, delta) - sqrt(del) ) / (norm(D)^2);
end