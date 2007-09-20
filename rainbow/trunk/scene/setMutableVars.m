function scene = setMutableVars(scene, varlist)

for n = 1:size(varlist,1)
    rowID = find( strcmp( scene.vars(:,1), varlist(n,1) ) );
    bodyID = scene.vars{rowID, 2};
    
    % Set value
    %scene.bodyData(bodyID);
    scene.bodyData(bodyID).validWorldPose = false;
    
    % Walk along all distal bodies and set there validWorldPose to false
    distBodyToVisit = scene.bodyData(bodyID).distBodyIDs;
    while ( ~isempty(distBodyToVisit) )
        distBodyID = distBodyToVisit(1);
        scene.bodyData(distBodyID).validWorldPose = false;
        distBodyToVisit = [distBodyToVisit; scene.bodyData(distBodyID).distBodyIDs];
        distBodyToVisit = distBodyToVisit(2:end,1);
    end
end