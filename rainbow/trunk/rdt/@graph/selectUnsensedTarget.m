function Tj = selectUnsensedTarget(G, id, targets, minEff)

% Get shortest path to id.
[path, dist, pathEff] = get_shortest_path(G, id);

% First find all targets with times greater than current node
t_node = G.node_data(end,id);
futureTargetIDs = find( targets(end,:) > t_node );

if ( isempty(futureTargetIDs) )
    Tj = [];
else
%     % Of these future targets, find either the first one with a sensed rating
%     % less than minEff or simply the least sensed one.
%     %
%     %   TODO: explore the idea of probalistically selecting the future
%     %   target based on its time and/or effectiveness so far.
%     futurePathEff = pathEff(futureTargetIDs);
%     ind = find( futurePathEff <= minEff );
%     
%     j = 0;
%     if ( ~isempty(ind) )
%         % ind is not empty which means there is a future target with an
%         % effectiveness less than our minimum. Therefore just select the
%         % first target that doesn't meet our criteria.
%         j = futureTargetIDs(ind(1));
%     else
%         % ind is empty so there are no targets with an effectiveness less
%         % than our required minimum. In this case, simply select the target
%         % with the smallest effectiveness.
%         ind = find( futurePathEff == min(futurePathEff) );
%         j = futureTargetIDs(ind);
%     end
%     
%     Tj = targets(:,j);
    Tj = targets(:,futureTargetIDs(1));
end