function [scene, T_world_body] = computeWorldTransform(scene, name)

% Find body of interest
bodyID = -1;
if (ischar(name))
    bodyRowID = find( strcmp(scene.bodyIDs(:,1), name ) );
    bodyID = scene.bodyIDs{bodyRowID, 2};
else
    bodyID = name;
end

if (scene.bodyData(bodyID).validWorldPose)
    % If body already has a valid pose, we are ok, just return
    T_world_body = scene.bodyData(bodyID).T_world_body;
    return;
else
    if ( scene.bodyData(bodyID).proxBodyID == -1 )
        % Else, if the parent to this body is the world, then T_world_body =
        % T_prox_body.
        scene.bodyData(bodyID).validWorldPose = true;
        scene.bodyData(bodyID).T_world_body = scene.bodyData(bodyID).T_prox_body;
        T_world_body = scene.bodyData(bodyID).T_prox_body;
    else
        % Now our only choice is to recurse until we find a valid world
        % pose
        [scene, T_world_prox] = computeWorldTransform(scene, scene.bodyData(bodyID).proxBodyID);
        T_world_body = T_world_prox * scene.bodyData(bodyID).T_prox_body;
        
        scene.bodyData(bodyID).validWorldPose = true;
        scene.bodyData(bodyID).T_world_body = T_world_body;
    end
end
        