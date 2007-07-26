function scene = buildScene(scene, q)

for n = 1:length(scene.space)
	scene.space{n} = buildSpace(scene.space{n});
end
% -------------------------------------------------------------------------
% DEPRECATED
% c2 = cos(theta + theta_off);
% s2 = sin(theta + theta_off);
% c12 = cos(alpha);
% s12 = sin(alpha);
% 
% T = [c2, -s2, 0, a; s2*c12, c2*c12, -s12, -s12*S; s2*s12, c2*s12, c12, c12*S];
% p = T(1:3,4);
% R = T(1:3,1:3);
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% DEPRECATED
% c = cos(ang);
% s = sin(ang);
% v = 1 - c;
% m = m/norm(m);
% 
% R = [v*m(1)^2 + c, m(1)*m(2)*v - m(3)*s, m(1)*m(3)*v + m(2)*s;
%      m(1)*m(2)*v + m(3)*s, v*m(2)^2 + c, m(2)*m(3)*v - m(1)*s;
%      m(1)*m(3)*v - m(2)*s, m(2)*m(3)*v + m(1)*s, v*m(3)^2 + c];
% -------------------------------------------------------------------------

end


% -------------------------------------------------------------------------
% Build space
% -------------------------------------------------------------------------
function space = buildSpace(s)
space = s;
space.id = CreateHashSpace(0);

for b = 1:length(s.body)
    body = s.body{b};
        
    T_wcs_body = eye(4);
    if (~strcmp(body.base,'world'))
        
    end
    [T_wcs_base,bfound] = getBaseBody(body.base, scene.space(s).body(1:b));
    if ~bfound, warning('Base transform for %s not found: %s', body.name, body.base); end

    T_base_body = eye(4); T_base_body(1:3,4) = body.pos; T_base_body(1:3,1:3) = body.RMatrix;
    T_wcs_body = T_wcs_base * T_base_body;

    % Update scene structure
    scene.space(s).body(b).pos = T_wcs_body(1:3,4);
    scene.space(s).body(b).RMatrix = T_wcs_body(1:3,1:3);

    for g = 1:length(body.geometry)
        geom = body.geometry(g);
        T_body_geom = eye(4); T_body_geom(1:3,4) = geom.pos; T_body_geom(1:3,1:3) = geom.RMatrix;
        T_wcs_geom = T_wcs_body * T_body_geom;

        % Update scene structure
        scene.space(s).body(b).geometry(g).pos = T_wcs_geom(1:3,4);
        scene.space(s).body(b).geometry(g).RMatrix = T_wcs_geom(1:3,1:3);
    end
end
end

% -------------------------------------------------------------------------
% Post-process scene
% -------------------------------------------------------------------------
function scene = postProcess(scene)
% For each space, walk down the tree of bodies and geoms making each
% relative to WCS. It is assumed that the first body of every
for s = 1:length(scene.space)
    for b = 1:length(scene.space(s).body)
        body = scene.space(s).body(b);
        [T_wcs_base,bfound] = getBaseTransform(body.base, scene.space(s).body(1:b));
        if ~bfound, warning('Base transform for %s not found: %s', body.name, body.base); end

        T_base_body = eye(4); T_base_body(1:3,4) = body.pos; T_base_body(1:3,1:3) = body.RMatrix;
        T_wcs_body = T_wcs_base * T_base_body;
        
        % Update scene structure
        scene.space(s).body(b).pos = T_wcs_body(1:3,4);
        scene.space(s).body(b).RMatrix = T_wcs_body(1:3,1:3);
        
        for g = 1:length(body.geometry)
            geom = body.geometry(g);
            T_body_geom = eye(4); T_body_geom(1:3,4) = geom.pos; T_body_geom(1:3,1:3) = geom.RMatrix;
            T_wcs_geom = T_wcs_body * T_body_geom;
            
            % Update scene structure
            scene.space(s).body(b).geometry(g).pos = T_wcs_geom(1:3,4);
            scene.space(s).body(b).geometry(g).RMatrix = T_wcs_geom(1:3,1:3);
        end
    end
end

end

% -------------------------------------------------------------------------
% Get parent transform
% -------------------------------------------------------------------------
function [T,bfound] = getBaseTransform(base, sbody)
bfound = false;
T = eye(4);
if (base == 'world')
    bfound = true;
else
    for n=1:length(sbody)
        if (sbody(n).name == base)
            T(1:3,4) = sbody(n).pos;
            T(1:3,1:3) = sbody(n).RMatrix;
            bfound = true;
        end
    end
end
end