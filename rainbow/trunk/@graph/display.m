function display(G)
% Displays the contents of a Graph object
disp(' ');
disp([inputname(1),' = '])
disp(' ');
disp( ['   ','Number of nodes: ' num2str(size(G.node_data,2))] )
nedges = length( find( G.connectivity > 0 ) );
disp( ['   ','Number of edges: ' num2str(nedges)] )
disp(' ');
disp( ['   ','Paths invalid: ' num2str(G.pathsInvalid)] )