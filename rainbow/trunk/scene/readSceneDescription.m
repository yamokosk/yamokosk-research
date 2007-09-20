function scene = readSceneDescription(filename)

% PARSESCENE Load scene description from an XML file.
tree = [];
try
    tree = xmlread(filename);
catch
    error('Failed to read XML file %s.',filename);
end

% Find a list of all space elements.
allSpaceItems = tree.getElementsByTagName('space');
mutableVarList = [];
bodyNameIDList = [];
bodyCount = 1;
geomCount = 1;

try
    % Loop over all of the spaces
    for spaceIndex = 0:allSpaceItems.getLength-1
        thisSpaceItem = allSpaceItems.item(spaceIndex);
 
        % For the current space, get all of its bodies.. 
        allBodyItems = thisSpaceItem.getElementsByTagName('body');
        %spaceVarList = [];
        % .. and loop over them
        for bodyIndex = 0:allBodyItems.getLength-1
            thisBodyItem = allBodyItems.item(bodyIndex);
            thisBodyAttrib = parseAttributes(thisBodyItem);
            
            bodyNameIDList = [bodyNameIDList; {thisBodyAttrib.name}, bodyCount];
            bodyData(bodyCount) = struct('name',               thisBodyAttrib.name, ...
                                         'proxBodyID',         -1, ...
                                         'distBodyIDs',         [], ...
                                         'spaceID',            -1, ...
                                         'geomIDs',            [], ...
                                         'T_prox_body',        eye(4), ...
                                         'T_world_body',       eye(4), ...
                                         'transform',          [], ...
                                         'validWorldPose',     false);
            
            if (~strcmp(thisBodyAttrib.prox, 'world'))
                % Set parent ID of current body
                parentRowID = find( strcmp(bodyNameIDList(:,1), thisBodyAttrib.prox) );
                parentID = bodyNameIDList{parentRowID, 2};
                bodyData(bodyCount).proxBodyID = parentID;
                
                % In parent, set distal body ID
                bodyData(parentID).distBodyIDs = [bodyData(parentID).distBodyIDs; bodyCount];
            end
            
            % Next parse this body's transform tag.. there should only be one.
            [theTransformItem, numBodyTransforms] = getBodyTransformItem(thisBodyItem);
            
            if (numBodyTransforms > 1)
                error('Body %s has more than one transform tag!', thisBodyAttrib.name);
            end
            
            if (numBodyTransforms == 1)
                [transformStruct, mutableVarNames, T_prox_body] = parseTransform(theTransformItem);
                bodyData(bodyCount).transform = transformStruct;
                bodyData(bodyCount).T_prox_body = T_prox_body;
                
                numMutableVars = length(mutableVarNames);
                
                if (numMutableVars > 0)
                    mutableVarList = [mutableVarList; mutableVarNames, repmat({bodyCount}, numMutableVars, 1)];
                end
            end
            
            allGeomItems = thisBodyItem.getElementsByTagName('geometry');
            for geomIndex = 0:allGeomItems.getLength-1
                thisGeomItem = allGeomItems.item(geomIndex);
                thisGeomAttrib = parseAttributes(thisGeomItem);
                
                geomData(geomCount) = struct('name',            thisGeomAttrib.name, ...
                                             'type',            thisGeomAttrib.type, ...
                                             'bodyID',          bodyCount, ...
                                             'params',          [], ...
                                             'T_body_geom',     eye(4), ...
                                             'T_world_geom',    eye(4), ...
                                             'validWorldPose',  false);
                
                allParamItems = thisGeomItem.getElementsByTagName('parameter');
                for paramIndex = 0:allParamItems.getLength-1
                    thisParamItem = allParamItems.item(paramIndex);
                    geomData(geomCount).params{paramIndex+1} = parseAttributes(thisParamItem);                
                end % End param loop
                
                allTransformItems = thisGeomItem.getElementsByTagName('transform');
                if (allTransformItems.getLength > 1)
                    error('Geom %s on body %s has more than one transform tag!', thisGeomAttrib.name, thisBodyAttrib.name);
                end
                
                if (allTransformItems.getLength == 1)
                    geomTransformItem = allTransformItems.item(0);
                    [transformStruct, mutableVarNames, T_prox_body] = parseTransform(geomTransformItem);
                    geomData(geomCount).T_body_geom = T_prox_body;
                    
                    if (~isempty(mutableVarNames))
                        error('Geom %s has a mutable variable! Only bodies are allowed to have mutable variables', thisGeomAttrib.name);
                    end
                end
                bodyData(bodyCount).geomIDs = [bodyData(bodyCount).geomIDs; geomCount];
                
                geomCount = geomCount + 1;
            end % End geom loop
            bodyCount = bodyCount + 1;
        end % End body loop

    end
catch
    err = lasterror;
    error('Cannot parse %s. \n%s', filename, err.message);
    lasterror('reset');
end

scene.bodyIDs = bodyNameIDList;
scene.bodyData = bodyData;
scene.geomData = geomData;
scene.vars = mutableVarList;

% We have now parsed the XML file.. time to compute the body/geometry
% locations in world coordinates.

function [theTransformItem, numBodyTransforms] = getBodyTransformItem(thisBodyItem)

allChildItems = thisBodyItem.getChildNodes;
numBodyTransforms = 0;
theTransformItem = [];

for c = 0:allChildItems.getLength-1
    thisChildItem = allChildItems.item(c);
    
    if thisChildItem.getNodeType == thisChildItem.ELEMENT_NODE
        if (strcmp(char(thisChildItem.getNodeName), 'transform'))
            theTransformItem = thisChildItem;
            numBodyTransforms = numBodyTransforms + 1;
        end
    end
end



function [tStruct, mutableVarNames, T] = parseTransform(theNode)
allChildItems = theNode.getChildNodes;
T = eye(4); mutableVarNames = {};
numTransforms = 1; numMutableVars = 1;

for c = 0:allChildItems.getLength-1
    thisChildItem = allChildItems.item(c);
    
    if thisChildItem.getNodeType == thisChildItem.ELEMENT_NODE
        att = parseAttributes(thisChildItem);

        if (~isfield(att, 'mutable'))
            att.mutable = 'false';
        end
        
        if (~isfield(att, 'name'))
            att.name = '';
        end
        
        switch (char(thisChildItem.getNodeName))
            case 'translation'
                T = T * transl(parseValue(att.value));
                tStruct(numTransforms) = struct('type', 'translation', ... 
                                                'subtype', 'translation', ...
                                                'mutable', eval(att.mutable), ...
                                                'name', att.name, ...
                                                'value', parseValue(att.value));
            case 'rotation'
                switch (att.type)
                    case 'x'
                        T = T * rotx(parseValue(att.value));
                    case 'y'
                        T = T * roty(parseValue(att.value));
                    case 'z'
                        T = T * rotz(parseValue(att.value));
                    case 'e123'
                        v = parseValue(att.value);
                        T = T * rotx(v(1)) * roty(v(2)) * rotz(v(3));
                    case 'e123t'
                        v = parseValue(att.value);
                        T = T * rotx(-v(1)) * roty(-v(2)) * rotz(-v(3));
                    otherwise
                        error('Tag rotation has an unrecognized type: %s', att.type);
                end
                tStruct(numTransforms) = struct('type', 'rotation', ... 
                                                'subtype', att.type, ...
                                                'mutable', eval(att.mutable), ...
                                                'name', att.name, ...
                                                'value', parseValue(att.value));
            otherwise
                error('Tag transform has an unrecognized child tag: %s', char(thisChildItem.getNodeName));
        end
        
        % Record in a separate list the mutable variable names
        if (tStruct(numTransforms).mutable)
            mutableVarNames(numMutableVars,1) = {tStruct(numTransforms).name};
            numMutableVars = numMutableVars + 1;
        end
        numTransforms = numTransforms + 1;
        
    end
    
end



% ----- Subfunction PARSEATTRIBUTES -----
function theStruct = parseAttributes(varargin)
% Create attributes structure.
theNode = varargin{1};
if nargin == 1
    theStruct = [];
elseif nargin == 2
    theStruct = varargin{2};
end

if theNode.hasAttributes
   theAttributes = theNode.getAttributes;
   numAttributes = theAttributes.getLength;

   for count = 1:numAttributes
      attrib = theAttributes.item(count-1);
      theStruct.(char(attrib.getName)) = char(attrib.getValue);
   end
end


function v = parseValue(str)
v = [];
n = 0;
while(~isempty(str))
    n = n + 1;
    [token, str] = strtok(str);
    v(n) = eval(token);
end