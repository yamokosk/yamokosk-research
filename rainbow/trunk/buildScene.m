function scene = buildScene(scene, q)

if (size(q,1) < scene.num_variables)
    error('Not enough dynamic variables defined to build scene.');
elseif (size(q,1) > scene.num_variables)
    error('Too many dynamic variables defined to build scene.');
end

for n = 1:length(scene.space)
	if isfield(scene.space{n},'id') idSpace = scene.space{n}.id;
    else idSpace = CreateHashSpace(0); end

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

        % Create body in ODE if we have to
        if isfield(body,'id') idBody = body.id;
        else idBody = CreateBody(); end
                
        % Build the geoms
        if isfield(body,'geometry')
            
            for g = 1:length(body.geometry)
                geom = body.geometry{g};

                % Geoms are set and forget.. only bodies get updated.
                if ~isfield(geom, 'id')
                    
                    idSpaceTemp = 0; idGeom = 0;
                    if ~isfield(geom, 'transform'), idSpaceTemp = idSpace; end;

                    switch (geom.type)
                        case 'ccylinder'
                            idGeom = CreateCCylinder(idSpaceTemp, geom.radius, geom.length);
                        case 'sphere'
                            idGeom = CreateSphere(idSpaceTemp, geom.radius);
                        case 'box'
                            idGeom = CreateBox(idSpaceTemp, geom.length, geom.width, geom.height);
                        case 'plane'
                            idGeom = CreatePlane(idSpaceTemp, [geom.normal_x,geom.normal_y,geom.normal_z,geom.d]);
                            scene.space{n}.body{b}.geometry{g}.id = idGeom;
                            break;
                        otherwise
                            warning('%s is an unrecognized geometry type', geom.type);
                    end
                   
                    % Create relative transform if we need to
                    if isfield(geom, 'transform')
                        T_body_geom = computeTransform(geom.transform);
                        GeomSetPosition(idGeom, T_body_geom(1:3,4));
                        GeomSetRotation(idGeom, T_body_geom(1:3,1:3));

                        idTrans = CreateGeomTransform(idSpace);
                        GeomTransformSetGeom(idTrans, idGeom);
                        GeomSetBody(idTrans, idBody); % Allows us to set and forget geoms... 

                        % Update structure
                        scene.space{n}.body{b}.geometry{g}.transform.id = idTrans;
                    else
                        GeomSetBody(idGeom, idBody); % Allows us to set and forget geoms...
                    end

                    % Update structure
                    scene.space{n}.body{b}.geometry{g}.id = idGeom;
                end % End if ~isfield(geom,'id')             
            end
        end

        % Set body pos/rotation
        BodySetRotation(idBody, T_wcs_obj(1:3,1:3));
        BodySetPosition(idBody, T_wcs_obj(1:3,4));
        
        % Update structure
        scene.space{n}.body{b}.T_wcs_obj = T_wcs_obj;
        scene.space{n}.body{b}.id = idBody;
    end

    % Update structure
    scene.space{n}.id = idSpace;
end

end


% Get parent transform
function [T,bfound] = getBaseTransform(base, sbody)
bfound = false;
T = eye(4);
if strcmp(base,'world')
    bfound = true;
else
    for n=1:length(sbody)
        if (strcmp(sbody{n}.name, base))
            T = sbody{n}.T_wcs_obj;
            bfound = true;
        end
    end
end
end


function T = computeTransform(transform, varargin)
% Compute transform

q = [];
if nargin == 2
    q = varargin{1};
end

% If we have a dynamic variable, check that the user passed in the correct
% tuple.
if ~isfield(transform, 'type')
    warning('No transform type found!');
    fieldnames(transform)    
    T = eye(4);
    return
end

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
        T = getAffineParameters(transform,q);
        
%         c = cos(ang);
%         s = sin(ang);
%         v = 1 - c;
%         m = m/norm(m);
        
%         T(1:3,1:3) = [v*m(1)^2 + c, m(1)*m(2)*v - m(3)*s, m(1)*m(3)*v + m(2)*s;
%                       m(1)*m(2)*v + m(3)*s, v*m(2)^2 + c, m(2)*m(3)*v - m(1)*s;
%                       m(1)*m(3)*v - m(2)*s, m(2)*m(3)*v + m(1)*s,
%                       v*m(3)^2 + c];
    otherwise
        T = eye(4);
        warning('Unknown transformation type: %s', transform.type);
end
end


% -------------------------------------------------------------------------
% Get transform parameters
% -------------------------------------------------------------------------
function [a,alpha,S,theta,theta_off] = getDHParameters(t,dyn)

if isfield(t, 'static')
    static_fields = fieldnames(t.static);
else
    static_fields = [];
end

if isfield(t,'dynamic')
    dynamic_fields = fieldnames(t.dynamic);
else
    dynamic_fields = [];
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

for s = 1:length(dynamic_fields)
    str = dynamic_fields{s};
    name = t.dynamic.(str);
    ind = find( strcmp(dyn(:,1),name) );
    if isempty( ind )
        error('Variable %s is declared dynamic but no value for it was passed in.',name);
    end
    
    switch str
        case 'link_length'
            a = dyn{ind,2};
        case 'twist_angle'
            alpha = dyn{ind,2};
        case 'joint_offset'
            S = dyn{ind,2};
        case 'joint_angle'
            theta = dyn{ind,2};
        case 'joint_angle_offset'
            theta_off = dyn{ind,2};
    end
end

end

% -------------------------------------------------------------------------
% Get affine parametrs
% -------------------------------------------------------------------------
function T = getAffineParameters(t,q)

if isfield(t,'static')
    static_fields = fieldnames(t.static);
else
    static_fields = [];
end

if isfield(t,'dynamic')
    dynamic_fields = fieldnames(t.dynamic);
else
    dynamic_fields = [];
end

p = zeros(3,1);
R = eye(4);

for s = 1:length(static_fields)
    str = static_fields{s};
    switch str
        case 'translation'
            p = t.static.(str);
        case 'r123'
            ang = t.static.(str);
            R = rotx(ang(1))*roty(ang(2))*rotz(ang(3));
        case 'r213'
            ang = t.static.(str);
            R = roty(ang(1))*rotx(ang(2))*rotz(ang(3));
        case 'r312'
            ang = t.static.(str);
            R = rotz(ang(1))*rotx(ang(2))*roty(ang(3));
    end
end

for d = 1:length(dynamic_fields)
    str = dynamic_fields{d};
    name = t.dynamic.(str);
    ind = find( strcmp(q(:,1),name) );
    if isempty( ind )
        error('Variable %s is declared dynamic but no value for it was passed in.',name);
    end
    
    switch str
        case 'translation'
            p = q{ind,2};
        case 'r123'
            ang = q{ind,2};
            R = rotx(ang(1))*roty(ang(2))*rotz(ang(3));
        case 'r213'
            ang = q{ind,2};
            R = roty(ang(1))*rotx(ang(2))*rotz(ang(3));
        case 'r312'
            ang = q{ind,2};
            R = rotz(ang(1))*rotx(ang(2))*roty(ang(3));
    end
end

T = R;
T(1:3,4) = p;
end