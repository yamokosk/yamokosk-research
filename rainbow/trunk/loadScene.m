% LOADSCENE(FILENAME) - Load scene description from file
%   loadScene() reads in object locations and orientations which are stored
%   in an XML file. An example of XML structure this function will
%   successfully parse is listed below. This example creates one space
%   composed of two bodies. Each body has an associated geometry. Notice
%   that both bodies and geometries can have transforms.
%
%   EXAMPLE XML:
%   <scene units="mm">
%    	<space name="space_name">
%           <body name="body1" base="world">
%               <transform type="relative">
%                   <parameter type="translation" value="0 0 584.2+eps" />
%               </transform>
%               <geometry type="ccylinder" name="ccylinder01">
%                   <parameter type="length" value="200" />
%                   <parameter type="radius" value="112" />
%                   <transform type="relative">
%                       <parameter type="translation" value="0 0 -178.5" />
%                   </transform>
%               </geometry>
%       	</body>
%           <body name="body2" base="body1">
%               <transform type="dh">
%                   <parameter type="link_length" value="0" />
%                   <parameter type="twist_angle" value="-pi/2" />
%                   <parameter type="joint_offset" value="0" />
%                   <parameter type="joint_angle_offset" value="-pi/2" />
%               </transform>
%               <geometry type="ccylinder" name="ccylinder01">
%                   <parameter type="length" value="617.5" />
%                   <parameter type="radius" value="160" />
%                   <transform type="relative">
%                       <parameter type="translation" value="129 -30 0" />
%                       <parameter type="rotation" value="pi/2 0 1 0" />
%                   </transform>
%               </geometry>
%           </body>
%       </space>
%   </scene>
function scene = loadScene(filename)
xmlstr=fileread(filename);
scene_node = xml_parseany(xmlstr);
scene.units = '';

if isfield(scene_node, 'ATTRIBUTE')
    if isfield(scene_node.ATTRIBUTE, 'units')
        scene.units = scene_node.ATTRIBUTE.units;
    end
end

for n = 1:length(scene_node.space)
    scene.space{n} = parseSpace(scene_node.space{n});
end

% -------------------------------------------------------------------------
% DEPRECATED
% Scene is loaded from XML data.. now need to make all data relative to WCS
%scene = postProcess(scene);
% -------------------------------------------------------------------------

end

% -------------------------------------------------------------------------
% Parse a space
% -------------------------------------------------------------------------
function sspace = parseSpace(space)
sspace.name = '';

if isfield(space, 'ATTRIBUTE')
    if isfield(space.ATTRIBUTE, 'name')
        sspace.name = space.ATTRIBUTE.name;
    end
end

for n = 1:length(space.body) 
    sspace.body{n} = parseBody(space.body{n});
end

end

% -------------------------------------------------------------------------
% Parse a body
% -------------------------------------------------------------------------
function sbody = parseBody(body)
sbody.name = '';
sbody.base = 'world';

if isfield(body, 'ATTRIBUTE')
    if isfield(body.ATTRIBUTE, 'name')
        sbody.name = body.ATTRIBUTE.name;
    end
    if isfield(body.ATTRIBUTE, 'base')
        sbody.base = body.ATTRIBUTE.base;
    end
end

if isfield(body, 'transform')
    sbody.transform = parseTransform(body.transform{1});
end

if isfield(body, 'geometry')
    for n=1:length(body.geometry)
        sbody.geometry{n} = parseGeom(body.geometry{n});
    end
end

end

% -------------------------------------------------------------------------
% Parse a geom
% -------------------------------------------------------------------------
function sgeom = parseGeom(geom)
sgeom.type = '';
sgeom.name = '';

if isfield(geom, 'ATTRIBUTE')
    if isfield(geom.ATTRIBUTE, 'type')
        sgeom.type = geom.ATTRIBUTE.type;
    else
        errmsg('Geometry type not specified!');
    end
    if isfield(geom.ATTRIBUTE, 'name')
        sgeom.name = geom.ATTRIBUTE.name;
    end
end

if isfield(geom, 'transform')
    sgeom.transform = parseTransform(geom.transform{1});
end

if isfield(geom, 'parameter')
    for n=1:length(geom.parameter)
        sgeom.(geom.parameter{n}.ATTRIBUTE.type) = eval(geom.parameter{n}.ATTRIBUTE.value);
    end
end 
end

% -------------------------------------------------------------------------
% Parse a transform
% -------------------------------------------------------------------------
function stransform = parseTransform(transform)
transform_type = transform.ATTRIBUTE.type;

switch transform_type
    case 'dh'
        stransform = parseDH(transform);
    case 'relative'
        stransform = parseRelative(transform);
    otherwise
        warning('Encountered a transform type I have never seen before: %s', transform_type);
end

end % End of parseTransform()

% -------------------------------------------------------------------------
% Parse DH parameters
% -------------------------------------------------------------------------
function stransform = parseDH(transform)
for n = 1:length(transform.parameter)
    stransform.(transform.parameter{n}.ATTRIBUTE.type) = eval(transform.parameter{n}.ATTRIBUTE.value);
end

end % End of parseDH()

% -------------------------------------------------------------------------
% Parse relative transformation
% -------------------------------------------------------------------------
function stransform = parseRelative(transform)

for j = 1:length(transform.parameter)
    switch(transform.parameter{j}.ATTRIBUTE.type)
        case 'translation'
            str = transform.parameter{j}.ATTRIBUTE.value;
            p = zeros(3,1);
            for n=1:3
                if isempty(str), warning('Translation vector must have three elements: %s', transform.parameter{j}.ATTRIBUTE.value); end;
                [token, str] = strtok(str);
                p(n) = eval(token);
            end
            stransform.(transform.parameter{j}.ATTRIBUTE.type) = p;
        case 'rotation'
            % Get angle if it exists
            str = transform.parameter{j}.ATTRIBUTE.value;
            m = zeros(4,1);
            for n=1:4
                if isempty(str), warning('Rotation axis vector must have three elements: %s', transform.parameter{j}.ATTRIBUTE.value); end;
                [token, str] = strtok(str);
                m(n) = eval(token);
            end
            stransform.(transform.parameter{j}.ATTRIBUTE.type) = m;
    end
end

end