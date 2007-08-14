if ~libisloaded('MODE')
	error('Collision detection library is not currently loaded!');
end

numBubbles = 100;
space1 = CreateHashSpace(0);
space2 = CreateHashSpace(0);
b1 = zeros(numBubbles,1);
b2 = zeros(numBubbles,1);
rand('state',sum(100*clock))

% Generate Bubbles
ub1 = [1;.5;1]; lb1 = [-1;-1;-1];
ub2 = [1;1;1]; lb2 = [-1;-.5;-1];
for n=1:numBubbles
    b1(n) = CreateSphere(space1,rand(1,1)*(.075-.05) + .05);
    SetGeomPosition(b1(n), rand(3,1).*(ub1-lb1) + lb1);
    
    b2(n) = CreateSphere(space2,rand(1,1)*(.075-.05) + .05);
    SetGeomPosition(b2(n), rand(3,1).*(ub2-lb2) + lb2); 
end

% Do collision test
tic;
CollisionPairs = SpaceCollide2(space1, space2)
stop = toc;
disp(['Total calculation time: ' num2str(stop)]);

% Plot spheres
% Figure creation
h = figure();
title('Bubble World!')
hold on;

for n=1:numBubbles
    b1Sphere = b1(n);
    b1Pos = GetGeomPosition(b1Sphere);
    b1Rad = GetGeomSphereRadius(b1Sphere);
    
    if ~isempty( find( CollisionPairs(:,1) == b1Sphere ) )
        drawSphere(b1Pos, b1Rad, 'FaceColor', 'r');
    else
        drawSphere(b1Pos, b1Rad, 'FaceColor', 'b');
    end
    
    b2Sphere = b2(n);
    b2Pos = GetGeomPosition(b2Sphere);
    b2Rad = GetGeomSphereRadius(b2Sphere);
    if ~isempty( find( CollisionPairs(:,2) == b2Sphere ) )
        drawSphere(b2Pos, b2Rad, 'FaceColor', 'r');
    else
        drawSphere(b2Pos, b2Rad, 'FaceColor', 'g');
    end
end

% for n=1:size(CollisionPairs,1)
%     b1Sphere = CollisionPairs(n,1);
%     b1Pos = GetGeomPosition(b1Sphere);
%     b1Rad = GetGeomSphereRadius(b1Sphere);
%     drawSphere(b1Pos, b1Rad, 'FaceColor', 'g');
%     
%     b2Sphere = CollisionPairs(n,2);
%     b2Pos = GetGeomPosition(b2Sphere);
% 	b2Rad = GetGeomSphereRadius(b2Sphere);
%     drawSphere(b2Pos, b2Rad, 'FaceColor', 'b');
% end

SpaceDestroy(space1);
SpaceDestroy(space2);
clear b1 b2 space1 space2 numBubbles CollisionPairs
clear all