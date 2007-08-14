function scene = drawScene(scene)

if isfield(scene, 'disp')
    if ~ishandle(scene.disp) 
        disp = createWindow();
    else
        disp = scene.disp;
    end
else
    disp = createWindow();
end

for n = 1:length(scene.space) % For all the spaces
    for b = 1:length(scene.space{n}.body) % For all the bodies
        body = scene.space{n}.body{b};
        
        % Get body id
        if isfield(body,'id') 
            idBody = body.id;
        else
            warning([body.name ' has no ID! Skipping it.']);
            continue;
        end
        
        % For the body, draw all of its geoms
        if isfield(body,'geometry')
            
            for g = 1:length(body.geometry)
                geom = body.geometry{g};

                % Geoms are set and forget.. only bodies get updated.
                if isfield(geom, 'id')
                    idGeom = geom.id;
                    gclass = GeomGetClass(idGeom);
                    R = GeomGetRotation(idGeom);
                    pos = GeomGetPosition(idGeom);
                    T = eye(4); T(1:3,1:3) = R; T(1:3,4) = pos;
                    
                    switch (gclass)
                        case 0 % Sphere
                            r = GeomSphereGetRadius(idGeom);
                            idGeom = createSphere(idSpaceTemp, geom.radius, geom.length);
                        case 1 % Box
                            idGeom = CreateSphere(idSpaceTemp, geom.radius);
                        case 2 % Capsule
                            idGeom = CreateBox(idSpaceTemp, geom.length, geom.width, geom.height);
                        case 3 % Plane
                            idGeom = CreatePlane(idSpaceTemp, [geom.normal_x,geom.normal_y,geom.normal_z,geom.d]);
                            scene.space{n}.body{b}.geometry{g}.id = idGeom;
                            break;
                        case 4 % Transform
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

function disp = createWindow()
disp.fig = figure('Renderer', 'OpenGL');

% X-Y trajectory
fig.XYaxes = axes('position',           [.1  .7  .8  .2]);

% Scene axes
disp.glaxes = axes('position',          [.1  .1  .8  .6], ...
                   'CameraPosition',    [36,60,48], ...
                   'CameraTarget',      [0,0,24], ...
                   'CameraUpVector',    [0,0,1], ...
                   'GridLineStyle',     '--');
               
end % End createWindow()