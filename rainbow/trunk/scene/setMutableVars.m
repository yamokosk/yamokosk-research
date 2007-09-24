function scene = setMutableVars(scene, varlist)

distBodyToVisit = [];

for n = 1:size(varlist,1)
    rowID = find( strcmp( scene.vars(:,1), varlist{n,1} ) );
    if (isempty(rowID))
        warning('Variable ''%s'' does not exist in scene description.', varlist{n,1});
        continue;
    end
    bodyID = scene.vars{rowID, 2};
    
    % Set value
    %scene.bodyData(bodyID);
    for m = 1:length(scene.bodyData(bodyID).transform)
        if ( strcmp(scene.bodyData(bodyID).transform(m).name, varlist{n,1}) )
            scene.bodyData(bodyID).transform(m).value = varlist{n,2};
            break;
        end
    end
    scene = computeLocalTransform(scene,bodyID);
    scene.bodyData(bodyID).validWorldPose = false;
    
    distBodyToVisit = union(distBodyToVisit, scene.bodyData(bodyID).distBodyIDs);
end

% Walk along all distal bodies and set there validWorldPose to false
while ( ~isempty(distBodyToVisit) )
    distBodyID = distBodyToVisit(1);
    scene.bodyData(distBodyID).validWorldPose = false;

    distBodyToVisit = union(distBodyToVisit, scene.bodyData(distBodyID).distBodyIDs);
    distBodyToVisit = setxor(distBodyID, distBodyToVisit);
end