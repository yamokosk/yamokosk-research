function scene = updateScene(scene)

for n = 1:length(scene.bodyData)
    scene = computeWorldTransform(scene, n);
end