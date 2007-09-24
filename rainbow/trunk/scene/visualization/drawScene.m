function scene = drawScene(scene)

scene = updateScene(scene);

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

for n = 1:length(scene.geomData) % For all the geoms
    body = scene.bodyData(scene.geomData{n}.bodyID);
    scene.geomData{n}.T_world_geom = body.T_world_body * scene.geomData{n}.T_body_geom;
    
    [fv,color] = createGeom(scene.geomData{n});
    
    if ishandle(scene.geomData{n}.ghandle)
        set(scene.geomData{n}.ghandle, 'Vertices', fv.vertices);
    else
        scene.geomData{n}.ghandle = patch(fv, 'FaceColor', color);
    end
end

% Update structure
scene.disp = disp;

end % End drawScene()


function [fv, c] = createGeom(geom)

% Quickly handle if it is a plane
if ( strcmp(geom.type, 'plane') )
    fv = createPlane([-2, 2, -4, 4], [geom.params.normal_x,geom.params.normal_y,geom.params.normal_z,geom.params.d]);
    c = [0,0,1];
    return;
end

switch (geom.type)
    case 'sphere'
        fv = createSphere(geom.T_world_geom, geom.params.radius);
        c = [1,0,0];
    case 'box'
        fv = createBox(geom.T_world_geom, [geom.params.length, geom.params.width, geom.params.height]);
        c = [1,1,0];
    case 'ccylinder'
        fv = createCapsule(geom.T_world_geom, geom.params.length, geom.params.radius);
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