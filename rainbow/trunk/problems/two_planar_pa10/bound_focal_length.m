function [fmin, fmax] = bound_focal_length(theta,d,panelArea,r_not)

Amin = 0.3 * panelArea; Amax = 0.8 * panelArea;
gamma = atan(r_not/d);

fmin = sqrt( Amin / (2 * (r_not/d) * (tan(theta + gamma) - tan(theta - gamma))) );
fmax = sqrt( Amax / (2 * (r_not/d) * (tan(theta + gamma) - tan(theta - gamma))) );

if ( fmin > fmax )
    temp = fmax;
    fmax = fmin;
    fmin = temp;
end

if ( fmin < 100 )
    fmin = 100;
end

fmin = fmin * 0.3937;
fmax = fmax * 0.3937;