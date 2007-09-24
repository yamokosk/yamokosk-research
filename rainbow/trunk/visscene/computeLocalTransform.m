function [scene,T_prox_body] = computeLocalTransform(scene,name)

% Find body of interest
bodyID = -1;
if (ischar(name))
    bodyRowID = find( strcmp(scene.bodyIDs(:,1), name ) );
    bodyID = scene.bodyIDs{bodyRowID, 2};
else
    bodyID = name;
end

T_prox_body = eye(4);

for n = 1:length(scene.bodyData(bodyID).transform)
    transStruct = scene.bodyData(bodyID).transform(n);
    switch (transStruct.type)
        case 'translation'
            T_prox_body = T_prox_body * transl(transStruct.value);
        case 'rotation'
            x = 0; y = 0; z = 0;
            switch (transStruct.subtype)
                case 'x'
                    x = transStruct.value;
                case 'y'
                    y = transStruct.value;
                case 'z'
                    z = transStruct.value;
                case 'e123'
                    x = transStruct.value(1);
                    y = transStruct.value(2);
                    z = transStruct.value(3);
                case 'e123t'
                    x = -transStruct.value(1);
                    y = -transStruct.value(2);
                    z = -transStruct.value(3);
                otherwise
                    error('Tag rotation has an unrecognized type: %s', att.type);
            end
            T_prox_body = T_prox_body * rotx(x) * roty(y) * rotz(z);
    end 
end

scene.bodyData(bodyID).T_prox_body = T_prox_body;