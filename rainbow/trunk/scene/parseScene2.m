function sceneStruct = parseScene(filename)

% PARSESCENE Load scene description from an XML file.
tree = [];
try
    tree = xmlread(filename);
catch
    error('Failed to read XML file %s.',filename);
end

% Recurse over child nodes.
% try
%     theStruct = parseChildNodes(tree);
% catch
%     error('Unable to parse XML file %s.', filename);
%     rethrow(lasterror);
% end

% Find a list of all space elements.
allSpaceItems = tree.getElementsByTagName('space');
try
for k = 0:allSpaceItems.getLength-1
    thisSpaceItem = allSpaceItems.item(k);
    sceneStruct.spaces{k+1} = parseSpace(thisSpaceItem);
end
catch
    err = lasterror;
    error('Cannot parse %s. \n%s', filename, err.message);
    lasterror('reset');
end


function spaceStruct = parseSpace(spaceItem);

att = parseAttributes(spaceItem);
spaceStruct = struct('name',        att.name, ...
                     'id',          -1);

allBodyItems = spaceItem.getElementsByTagName('body');
for m = 0:allBodyItems.getLength-1
    thisBodyItem = allBodyItems.item(m);
        
    spaceStruct.bodies{m+1} = parseBody(thisBodyItem);
end % End body loop


                 
function bodyStruct = parseBody(bodyItem)

att = parseAttributes(bodyItem);
if (~isfield(att, 'name'))
    error('A body tag does not have a name. Please check the XML description file.');
end

bodyStruct = struct('name',         att.name, ...
                    'parent',       'world', ... 
                    'id',           -1, ...
                    'T_prox_body',  eye(4));

if (isfield(att, 'parent'))
    bodyStruct.parent = att.parent;
end

allChildItems = bodyItem.getChildNodes;
numBodyTransforms = 0; numGeoms = 1;
for c = 0:allChildItems.getLength-1
    thisChildItem = allChildItems.item(c);
    
    if thisChildItem.getNodeType == thisChildItem.ELEMENT_NODE
        switch (char(thisChildItem.getNodeName))
            case 'transform'
                bodyStruct.T_prox_body = parseTransform(thisChildItem);
                numBodyTransforms = numBodyTransforms + 1;
                
                if (numBodyTransforms > 1)
                    error('Tag body (%s) has too many transform tags', att.name);
                end
            case 'geometry'
                bodyStruct.geoms{numGeoms} = parseGeom(thisChildItem);
                numGeoms = numGeoms + 1;
            otherwise
                error('Tag body (%s) has an unrecognized child tag: %s', att.name, char(thisChildItem.getNodeName));
        end
    end
end


function geomStruct = parseGeom(geomItem)
att = parseAttributes(geomItem);
if (~isfield(att, 'name'))
    error('A geom tag does not have a name. Please check the XML description file.');
end

geomStruct = struct('name',         att.name, ...
                    'id',           -1, ...
                    'T_prox_body',  eye(4), ...
                    'type',         att.type);
allParamItems = geomItem.getElementsByTagName('parameter');
for n=0:allParamItems.getLength-1
    thisParamItem = allParamItems.item(n);
    geomStruct = parseAttributes(thisParamItem,geomStruct);
end
    



% function T = parseTransform(theNode)
% allChildItems = theNode.getChildNodes;
% T = eye(4);
% for c = 0:allChildItems.getLength-1
%     thisChildItem = allChildItems.item(c);
%     
%     if thisChildItem.getNodeType == thisChildItem.ELEMENT_NODE
%         att = parseAttributes(thisChildItem);
% 
%         switch (char(thisChildItem.getNodeName))
%             case 'translation'
%                 T = T * transl(parseValue(att.value));
%             case 'rotation'
%                 switch (att.type)
%                     case 'x'
%                         T = T * rotx(parseValue(att.value));
%                     case 'y'
%                         T = T * roty(parseValue(att.value));
%                     case 'z'
%                         T = T * rotz(parseValue(att.value));
%                     case 'e123'
%                         v = parseValue(att.value);
%                         T = T * rotx(v(1)) * roty(v(2)) * rotz(v(3));
%                     case 'e123t'
%                         v = parseValue(att.value);
%                         T = T * rotx(-v(1)) * roty(-v(2)) * rotz(-v(3));
%                     otherwise
%                         error('Tag rotation has an unrecognized type: %s', att.type);
%                 end
%             otherwise
%                 error('Tag transform has an unrecognized child tag: %s', char(thisChildItem.getNodeName));
%         end
%     end
%     
% end

function [tStruct, T] = parseTransform(theNode)
allChildItems = theNode.getChildNodes;
T = eye(4); numTransforms = 1;
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
                                                'name', att.name
                                                'value', parseValue(att.value));
            case 'rotation'
%                 switch (att.type)
%                     case 'x'
%                         T = T * rotx(parseValue(att.value));
%                     case 'y'
%                         T = T * roty(parseValue(att.value));
%                     case 'z'
%                         T = T * rotz(parseValue(att.value));
%                     case 'e123'
%                         v = parseValue(att.value);
%                         T = T * rotx(v(1)) * roty(v(2)) * rotz(v(3));
%                     case 'e123t'
%                         v = parseValue(att.value);
%                         T = T * rotx(-v(1)) * roty(-v(2)) * rotz(-v(3));
%                     otherwise
%                         error('Tag rotation has an unrecognized type: %s', att.type);
%                 end
                tStruct(numTransforms) = struct('type', 'rotation', ... 
                                                'subtype', att.type, ...
                                                'mutable', eval(att.mutable), ...
                                                'name', att.name
                                                'value', parseValue(att.value));
            otherwise
                error('Tag transform has an unrecognized child tag: %s', char(thisChildItem.getNodeName));
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