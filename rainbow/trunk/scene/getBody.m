function b = getBody(scene, body_name)
for m=1:length(scene.space)
    for n=1:length(scene.space{m}.body)
        if (strcmp(scene.space{m}.body{n}.name, body_name))
            b = scene.space{m}.body{n};
        end
    end
end