function display(G)
% Displays the contents of a Graph object
disp(' ');
disp([inputname(1),' = '])
disp(' ');
disp( ['   ','Number of nodes: ' num2str(G.NodeCount)] )
disp( ['   ','Number of edges: ' num2str(G.EdgeCount)] )
disp(' ');
disp( ['   ','Paths invalid: ' num2str(G.ShortestPathsInvalid)] )