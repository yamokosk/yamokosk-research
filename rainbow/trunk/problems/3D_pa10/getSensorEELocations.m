function X = getSensorEELocations(nodes,udata)


[junk,c] = size(nodes);
X = zeros(3,c);

for n = 1:c
    [junk, x_sen] = metricToJointSpaceMap(nodes(:,n), udata);
    
    T = kine(udata.rsen,x_sen(1:6,1));
    X(:,n) = T(1:3,4);
end
