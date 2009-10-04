function k = irotk(T)

% Pull out rotation matrix
R = T(1:3,1:3);

ct = (trace(R) - 1)/2;
epsilon = 1e-3;

% Special case - theta is close to 0 or pi (or ct close to +/- 1)
if ( (abs(ct + 1) < epsilon) || (abs(ct - 1) < epsilon) )
    mx = sign( R(3,2) - R(2,3) ) * sqrt( (R(1,1) - ct) / (1 - ct) );
    my = sign( R(1,3) - R(3,1) ) * sqrt( (R(2,2) - ct) / (1 - ct) );
    mz = sign( R(2,1) - R(1,2) ) * sqrt( (R(3,3) - ct) / (1 - ct) );
    
    k = [mx; my; mz];
    [junk,I] = sort(abs(k),1,'descend');
    
    switch (I(1))
        case 1  % mx is the largest
            my = (R(1,2) + R(2,1))/(2*mx*(1-ct));
            mz = (R(1,3) + R(3,1))/(2*mx*(1-ct));
        case 2  % my is the largest
            mx = (R(1,2) + R(2,1))/(2*my*(1-ct));
            mz = (R(2,3) + R(3,2))/(2*my*(1-ct));
        case 3  % mz is the largest
            mx = (R(1,3) + R(3,1))/(2*mz*(1-ct));
            my = (R(2,3) + R(3,2))/(2*mz*(1-ct));
    end
    t = acos(ct);
else
    % Nominal case - theta = (0,pi)
    t = acos(ct);
    mz = (R(2,1) - R(1,2)) / 2*sin(t);
    my = (R(1,3) - R(3,1)) / 2*sin(t);
    mx = (R(3,2) - R(2,3)) / 2*sin(t);
end

k = [mx; my; mz];
k = k / norm(k);
k = k*t;