function scene = drawScene(scene)

if isfield(scene, 'disp')
    if ~ishandle(scene.disp.fig) 
        disp = createWindow();
    else
        disp = scene.disp;
        figure(disp.fig);
    end
else
    disp = createWindow();
end

set(gcf, 'CurrentAxes', disp.GLAxes);

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
                    [fv,color] = createGeom(geom);
                    
                    if isfield(geom, 'ghandle')
                        if ishandle(geom.ghandle)
                            set(geom.ghandle, 'Vertices', fv.vertices);
                        else
                            warning([geom.name ' did not have a valid handle.']);
                        end                            
                    else
                        scene.space{n}.body{b}.geometry{g}.ghandle = ...
                            patch(fv, 'FaceColor', color);
                    end
                    
                end
            end
        end
    end

    % Update structure
    scene.disp = disp;
end

end % End drawScene()


function [fv, c] = createGeom(geom)
id = geom.id;
gclass = dGeomGetClass(id);

% Quickly handle if it is a plane
if (gclass == 4) % Plane
    fv = createPlane([-2, 2, -4, 4], dGeomPlaneGetParams(id));
    c = [0,0,1];
    return;
end

T = eye(4);
T(1:3,1:3) = dGeomGetRotation(id);
T(1:3,4) = dGeomGetPosition(id);

if isfield(geom, 'transform')
    % Must get the transform's t-matrix
    T_wcs_trans = eye(4);
    T_wcs_trans(1:3,1:3) = dGeomGetRotation(geom.transform.id);
    T_wcs_trans(1:3,4) = dGeomGetPosition(geom.transform.id);
    T = T_wcs_trans * T;
end

switch (gclass)
    case 0 % Sphere
        fv = createSphere(T, dGeomSphereGetRadius(id));
        c = [1,0,0];
    case 1 % Box
        fv = createBox(T, dGeomBoxGetLengths(id));
        c = [1,1,0];
    case 2 % Capsule
        [r, len] = dGeomCCylinderGetParams(id);
        fv = createCapsule(T, len, r);
        c = [0,1,0];
     otherwise
        warning([geom.type ' is an unrecognized geometry type']);
end
end % End of drawGeom()


function disp = createWindow()
disp.fig = figure('Position', [50, 50, 1024, 768], 'Renderer', 'OpenGL');

% X-Y trajectory
%fig.XYAxes = axes('position',           [.1  .7  .8  .2]);

% Scene axes
disp.GLAxes = axes('DataAspectRatio',   [1, 1, 1], ...
                   'CameraPosition',    [.1,1,.6], ...
                   'CameraTarget',      [0,0,.5], ...
                   'CameraUpVector',    [0,0,1], ...
                   'Visible',           'off');
grid on;
               
end % End createWindow()