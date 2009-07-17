function T = rotk(k)

T = eye(4);

angle = norm(k);

if (angle ~= 0.0)
    vec = k / angle;
    kx = vec(1);
    ky = vec(2);
    kz = vec(3);
    s = sin(angle);
    c = cos(angle);
    v = 1-c;
    
    T(1,1) = kx*kx*v + c;
    T(1,2) = kx*ky*v - kz*s;
    T(1,3) = kx*kz*v + ky*s;
    
    T(2,1) = kx*ky*v + kz*s;
    T(2,2) = ky*ky*v + c;
    T(2,3) = ky*kz*v - kx*s;
    
    T(3,1) = kx*kz*v - ky*s;
    T(3,2) = ky*kz*v + kx*s;
    T(3,3) = kz*kz*v + c;
end