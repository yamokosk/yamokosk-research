V = G.V;
[dof, numNodes] = size(V);
minEff = 0.85;

% for n = 1:numNodes
%     [path, dist, eff] = get_shortest_path(G, n);
%     numSensed = length( find( eff > minEff ) );
%     if (numSensed == 6)
%         break;
%     end
% end

bestID = n;
bestPath = V(:,path);
udata.rsrc.ghandles = [];
udata.rsen.ghandles = [];
[dof, numNodes] = size(bestPath);
c=1;
for n = 1:8:numNodes
    plot_planar_pa10(bestPath(1:3,n), udata.rsrc);
    plot_planar_pa10(bestPath(7:9,n), udata.rsen);
    
    T_src=kine(udata.rsrc,bestPath(1:3,n),6);
    text(T_src(1,4),T_src(2,4)-0.05,num2str(c));
    
    T_sen=kine(udata.rsen,bestPath(7:9,n),6);
    text(T_sen(1,4),T_sen(2,4)+0.05,num2str(c));
    c = c+1;
end