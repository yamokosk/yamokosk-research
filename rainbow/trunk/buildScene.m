function scene = buildScene(scene, q)

if (size(q,1) < scene.num_variables)
    error('Not enough dynamic variables defined to build scene.');
elseif (size(q,1) > scene.num_variables)
    error('Too many dynamic variables defined to build scene.');
end

for n = 1:length(scene.space)
	%scene.space{n} = buildSpace(scene.space{n}, q);
    scene.space{n}.id = CreateHashSpace(0);

    for b = 1:length(scene.space{n}.body)
        body = scene.space{n}.body{b};
        [T_wcs_base,bfound] = getBaseTransform(body.base, scene.space{n}.body(1:b));
        if ~bfound, warning('Base transform for %s not found: %s', body.name, body.base); end

        % Update body wcs transform
        T_base_body = eye(4);
        if isfield(body, 'transform')
            T_base_body = computeTransform(body.transform, q);
        end
        T_wcs_obj = T_wcs_base * T_base_body;

        % Create body in ODE
        id = CreateBody();
        BodySetRotation(id, T_wcs_obj(1:3,1:3));
        BodySetPosition(id, T_wcs_obj(1:3,4));
        scene.space{n}.body{b}.transform.T_wcs_obj = T_wcs_obj;
        scene.space{n}.body{b}.id = id;

        % Compute bodies' geometry transforms in wcs
        for g = body.geometry
    %         T_body_geom = computeTransform(g.transform);
    %         T_wcs_geom = T_wcs_body * T_body_geom;
    % 
    %         % Create ODE Geom
    %         id = 0;
    %         switch (g.type)
    %             case 'ccylinder'
    %                 id = CreateCCylinder(space.id, g.radius, g.length);
    %             case 'sphere'
    %                 id = CreateSphere(space.id, g.radius);
    %             case 'box'
    %                 id = CreateSphere(space.id, g.length, g.width, g.heigth);
    %             case 'plane'
    %                 id = CreatePlane(space.id, [g.normal_x,g.normal.y,g.normalz,g.d]);
    %             otherwise
    %                 warning('%s is an unrecognized geometry type', g.type);
    %         end
    %         GeomSetPosition(id, T_wcs_geom(1:3,4));
    %         GeomSetRotation(id, T_wcs_geom(1:3,1:3));
    %         GeomSetBody(id, space.body{b}.id);
            % ODE steps
            %   1. Create the real geometry (aka, sphere, box, etc) but make it
            %   unattached to any space.. not sure how to do this right now.
            %   2. Create a transformed geometry and attach it to the proper
            %   space and body.
            %   3. Attach the real geometry to the transformed geometry, then
            %   set the real geometries position and rotation.
        end

    end

end

end

% -------------------------------------------------------------------------
% Post-process scene
% -------------------------------------------------------------------------
% function scene = postProcess(scene)
% % For each space, walk down the tree of bodies and geoms making each
% % relative to WCS. It is assumed that the first body of every
% for s = 1:length(scene.space)
%     for b = 1:length(scene.space(s).body)
%         body = scene.space(s).body(b);
%         [T_wcs_base,bfound] = getBaseTransform(body.base, scene.space(s).body(1:b));
%         if ~bfound, warning('Base transform for %s not found: %s', body.name, body.base); end
% 
%         T_base_body = eye(4); T_base_body(1:3,4) = body.pos; T_base_body(1:3,1:3) = body.RMatrix;
%         T_wcs_body = T_wcs_base * T_base_body;
%         
%         % Update scene structure
%         scene.space(s).body(b).pos = T_wcs_body(1:3,4);
%         scene.space(s).body(b).RMatrix = T_wcs_body(1:3,1:3);
%         
%         for g = 1:length(body.geometry)
%             geom = body.geometry(g);
%             T_body_geom = eye(4); T_body_geom(1:3,4) = geom.pos; T_body_geom(1:3,1:3) = geom.RMatrix;
%             T_wcs_geom = T_wcs_body * T_body_geom;
%             
%             % Update scene structure
%             scene.space(s).body(b).geometry(g).pos = T_wcs_geom(1:3,4);
%             scene.space(s).body(b).geometry(g).RMatrix = T_wcs_geom(1:3,1:3);
%         end
%     end
% end
% 
% end

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
        if (strcmp(sbody{n}.name, base))
            T = sbody{n}.transform.T_wcs_obj;
            bfound = true;
        end
    end
end
end

% -------------------------------------------------------------------------
% Compute transform
% -------------------------------------------------------------------------
function T = computeTransform(transform, varargin)
q = [];
if nargin == 2
    q = varargin{1};
end

% If we have a dynamic variable, check that the user passed in the correct
% tuple.
% find(strcmp(q(:,1),'q3'))
switch (transform.type)
    case 'dh'
        [a,alpha,S,theta,theta_off] = getDHParameters(transform,q);
        
        c2 = cos(theta + theta_off);
        s2 = sin(theta + theta_off);
        c12 = cos(alpha);
        s12 = sin(alpha);
        
        T = eye(4);
        T(1:3,1:4) = [c2, -s2, 0, a; s2*c12, c2*c12, -s12, -s12*S; s2*s12, c2*s12, c12, c12*S];
    case 'affine'
        [p,ang,m] = getAffineParameters(transform,q);
        
        c = cos(ang);
        s = sin(ang);
        v = 1 - c;
        m = m/norm(m);
        
        T = eye(4);
        T(1:3,1:3) = [v*m(1)^2 + c, m(1)*m(2)*v - m(3)*s, m(1)*m(3)*v + m(2)*s;
                      m(1)*m(2)*v + m(3)*s, v*m(2)^2 + c, m(2)*m(3)*v - m(1)*s;
                      m(1)*m(3)*v - m(2)*s, m(2)*m(3)*v + m(1)*s, v*m(3)^2 + c];
        T(1:3,4) = p;
    otherwise
        T = eye(4);
        warning('Unknown transformation type: %s', transform.type);
end
end

% -------------------------------------------------------------------------
% Get DH parametrs
% -------------------------------------------------------------------------
function [a,alpha,S,theta,theta_off] = getDHParameters(t,q)
static_fields = [];
if isfield(t, 'static')
    static_fields = fieldnames(t.static);
end

for s = 1:length(static_fields)
    str = static_fields{s};
    switch str
        case 'link_length'
            a = t.static.(str);
        case 'twist_angle'
            alpha = t.static.(str);
        case 'joint_offset'
            S = t.static.(str);
        case 'joint_angle'
            theta = t.static.(str);
        case 'joint_angle_offset'
            theta_off = t.static.(str);
    end
end

dynamic_fields = [];
if isfield(t,'dynamic')
    dynamic_fields = fieldnames(t.dynamic);
end

for d = 1:length(dynamic_fields)
    str = dynamic_fields{d};
    name = t.dynamic.(str);
    ind = find( strcmp(q(:,1),name) );
    if isempty( ind )
        error('Variable %s is declared dynamic but no value for it was passed in.',name);
    end
    
    switch str
        case 'link_length'
            a = q{ind,2};
        case 'twist_angle'
            alpha = q{ind,2};
        case 'joint_offset'
            S = q{ind,2};
        case 'joint_angle'
            theta = q{ind,2};
        case 'joint_angle_offset'
            theta_off = q{ind,2};
    end
end

end


% -------------------------------------------------------------------------
% Get affine parametrs
% -------------------------------------------------------------------------
function [p,ang,m] = getAffineParameters(t,q)
static_fields = [];
if isfield('static', t)
    static_fields = fieldnames(t.static);
end
p = zeros(3,1);
ang = 0;
m = [1;0;0];

for s = 1:length(static_fields)
    str = static_fields{s};
    switch str
        case 'translation'
            p = t.static.(str);
        case 'rotation'
            v = t.static.(str);
            ang = v(1);
            m = v(2:4);
    end
end

dynamic_fields = [];
if isfield('dynamic', t)
    dynamic_fields = fieldnames(t.dynamic);
end

for d = 1:length(dynamic_fields)
    str = dynamic_fields{d}
    name = t.dynamic.(str);
    ind = find( strcmp(q(:,1),name) );
    if isempty( ind )
        error('Variable %s is declared dynamic but no value for it was passed in.',name);
    end
    
    switch str
        case 'translation'
            p = q{ind,2};
        case 'rotation'
            v = q{ind,2};
            ang = v(1);
            m = v(2:4);
    end
end
end