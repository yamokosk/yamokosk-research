function plotResults(G,iter,doSetup)

if (doSetup == true)
    % Start a figure
    figure(1)
    set(1,'Position',[0,0,800,800]);
    hold on;
    grid on;
    %plot3(tdata(:,1),tdata(:,2), tdata(:,3),'k-');
    axis([-1.2 1.4 -0.7 0.7 0, 1]);
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    view(-45,45);
    drawnow;
    
    figure(2);
    return;
end
    
rootID = G.bestRootID;
leafID = G.bestLeafID;

[candPathIDs, candDist, candEff] = get_shortest_path(G, leafID);

nodes = G.V(candPathIDs);

figure(1);
child_handles = get(gca(),'Children');
types = get(child_handles,'Tag');
ind = find( strcmp(types, 'mylineplot') );
if ( ~isempty(ind) )
    set(child_handles(ind), 'XData', nodes(1,:), 'YData', nodes(2,:), 'ZData', nodes(3,:) );
    drawnow;
else
    h = plot3(nodes(1,:), nodes(2,:), nodes(3,:), '*--');
    set(h, 'Tag', 'mylineplot');
    drawnow;
end

figure(2)
hold on;
plot(iter,G.bestPathScore,'*');
drawnow;