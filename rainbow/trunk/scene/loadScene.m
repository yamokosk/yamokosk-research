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
%                   </transform>value="0 0 0" mutable="true"
%               </geometry>
%           </body>
%       </space>
%   </scene>
function scene = loadScene(filename)
xmlstr=fileread(filename);
scene_node = xml_parseany(xmlstr);
scene.lunitsratio = 1;
scene.runitsratio = 1;

if isfield(scene_node, 'ATTRIBUTE')
    if isfield(scene_node.ATTRIBUTE, 'lunits')
        scene.lunitsratio = unitsratio('m',scene_node.ATTRIBUTE.lunits);
    end
    if isfield(scene_node.ATTRIBUTE, 'runits')
        scene.runitsratio = unitsratio('rad',scene_node.ATTRIBUTE.runits);
    end
end

d = 0;
for n = 1:length(scene_node.space)
    [scene.space{n}, dyn] = parseSpace(scene_node.space{n});
    d = dyn + d;
end

scene.num_variables = d;
% -------------------------------------------------------------------------
% DEPRECATED
% Scene is loaded from XML data.. now need to make all data relative to WCS
%scene = postProcess(scene);
% -------------------------------------------------------------------------
end % End loadScene()

% -------------------------------------------------------------------------
% Parse a space
% -------------------------------------------------------------------------
function [sspace, dyn] = parseSpace(space)
sspace.name = '';

if isfield(space, 'ATTRIBUTE')
    if isfield(space.ATTRIBUTE, 'name')
        sspace.name = space.ATTRIBUTE.name;
    end
end

dyn = 0;
for n = 1:length(space.body)
    [sspace.body{n}, d] = parseBody(space.body{n});
    dyn = d + dyn;
end

end % End parseSpace

% -------------------------------------------------------------------------
% Parse a body
% -------------------------------------------------------------------------
function [sbody, dyn] = parseBody(body)
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

dyn = 0;
if isfield(body, 'transform')
    [sbody.transform, dyn] = parseTransform(body.transform{1});
end

if isfield(body, 'geometry')
    for n=1:length(body.geometry)
        sbody.geometry{n} = parseGeom(body.geometry{n});
    end
end

end % End parseBody()

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
        % HACK ALERT!!! - UNITS CRAP
        sgeom.(geom.parameter{n}.ATTRIBUTE.type) = eval(geom.parameter{n}.ATTRIBUTE.value)*1e-3;
    end
end
end % End parseGeom()

% -------------------------------------------------------------------------
% Parse a transform
% -------------------------------------------------------------------------
function [stransform, dyn] = parseTransform(transform)

% A transform can have several static and dynamic tags which take the form:
%   <static type="var_type" value="fix_value" />
%   <dynamic type="var_type" name="var_name" />
stransform.type = transform.ATTRIBUTE.type;
stransform.T_wcs_obj = eye(4);
dyn = 0;

if isfield(transform, 'static')
    for s = 1:length(transform.static)
        switch (transform.static{s}.ATTRIBUTE.type)
            case 'translation' % HACK ALERT - CRAPPY CODE AHEAD.. need to fix the units problem
                stransform.static.(transform.static{s}.ATTRIBUTE.type) = 1e-3 * parseValue(transform.static{s}.ATTRIBUTE.value);
            case 'r123'
                stransform.static.(transform.static{s}.ATTRIBUTE.type) = parseValue(transform.static{s}.ATTRIBUTE.value);
            case 't123'
                stransform.static.(transform.static{s}.ATTRIBUTE.type) = parseValue(transform.static{s}.ATTRIBUTE.value);
            case 'r321'
                stransform.static.(transform.static{s}.ATTRIBUTE.type) = parseValue(transform.static{s}.ATTRIBUTE.value);
            case 'r312'
                stransform.static.(transform.static{s}.ATTRIBUTE.type) = parseValue(transform.static{s}.ATTRIBUTE.value);
            case 'link_length'
                stransform.static.(transform.static{s}.ATTRIBUTE.type) = 1e-3 * parseValue(transform.static{s}.ATTRIBUTE.value);
            case 'joint_offset'
                stransform.static.(transform.static{s}.ATTRIBUTE.type) = 1e-3 * parseValue(transform.static{s}.ATTRIBUTE.value);
            case 'twist_angle'
                stransform.static.(transform.static{s}.ATTRIBUTE.type) = parseValue(transform.static{s}.ATTRIBUTE.value);
            case 'joint_angle'
                stransform.static.(transform.static{s}.ATTRIBUTE.type) = parseValue(transform.static{s}.ATTRIBUTE.value);
            case 'joint_angle_offset'
                stransform.static.(transform.static{s}.ATTRIBUTE.type) = parseValue(transform.static{s}.ATTRIBUTE.value);          
        end
    end
end

if isfield(transform, 'dynamic')
    dyn = length(transform.dynamic);
    for d = 1:dyn
        stransform.dynamic.(transform.dynamic{d}.ATTRIBUTE.type) = transform.dynamic{d}.ATTRIBUTE.name;
    end
end

    % -------------------------------------------------------------------------
    % DEPRECATED
    %
    % switch transform_type
    %     case 'dh'
    %         stransform = parseDH(transform);
    %     case 'affine'
    %         stransform = parseAffine(transform);
    %     otherwise
    %         warning('Encountered a transform type I have never seen before: %s', transform_type);
    % end
    % ---------------------------------------------------------------------

end % End of parseTransform()

% -------------------------------------------------------------------------
% Parse a value
% -------------------------------------------------------------------------
function v = parseValue(str)
v = [];
n = 0;
while(~isempty(str))
    n = n + 1;
    [token, str] = strtok(str);
    v(n) = eval(token);
end

end % End of parseValue()

% -------------------------------------------------------------------------
% DEPRECATED: Parse DH parameters
% % -------------------------------------------------------------------------
% function stransform = parseDH(transform)
% for n = 1:length(transform.parameter)
%     stransform.(transform.parameter{n}.ATTRIBUTE.type) = eval(transform.parameter{n}.ATTRIBUTE.value);
% end
%
% end % End of parseDH()

% -------------------------------------------------------------------------
% DEPRECATED: Parse relative transformation
% -------------------------------------------------------------------------
% function stransform = parseRelative(transform)
%
% for j = 1:length(transform.parameter)
%     switch(transform.parameter{j}.ATTRIBUTE.type)
%         case 'translation'
%             str = transform.parameter{j}.ATTRIBUTE.value;
%             p = zeros(3,1);
%             for n=1:3
%                 if isempty(str), warning('Translation vector must have three elements: %s', transform.parameter{j}.ATTRIBUTE.value); end;
%                 [token, str] = strtok(str);
%                 p(n) = eval(token);
%             end
%             stransform.(transform.parameter{j}.ATTRIBUTE.type) = p;
%         case 'rotation'
%             % Get angle if it exists
%             str = transform.parameter{j}.ATTRIBUTE.value;
%             m = zeros(4,1);
%             for n=1:4
%                 if isempty(str), warning('Rotation axis vector must have three elements: %s', transform.parameter{j}.ATTRIBUTE.value); end;
%                 [token, str] = strtok(str);
%                 m(n) = eval(token);
%             end
%             stransform.(transform.parameter{j}.ATTRIBUTE.type) = m;
%     end
% end
%
% end