function G = graph(N)
% Generic graph class
%   G = graph;      Creates an empty graph.
%   G = graph(N)    Creates an empty graph where the nodes have degree N.
%   G2 = graph(G1)  Copies G1 into G2.
if nargin == 0
    G.root_ids = [];
	G.node_data = [];
    G.node_weights = [];
    G.node_effectiveness = [];
    G.edge_weights = sparse(1,1);
	G.connectivity = sparse(1,1);
    G.pathsInvalid = true;
    G.pathDistances = [];
    G.pathPredecessors = [];
    G.bestPathScore = 0;
    G.bestLeafID = 0;
    G.bestRootID = 0;
	G = class(G,'graph');
elseif nargin == 1
    if ( isa(N,'graph') )
        G.root_ids = N.root_ids;
    	G.node_data = N.node_data;
        G.node_weights = N.node_weights;
        G.node_effectiveness = N.node_effectiveness;
        G.edge_weights = N.edge_weights;
        G.connectivity = N.connectivity;
        G.pathsInvalid = N.pathsInvalid;
        G.pathDistances = N.pathDistances;
        G.pathPredecessors = N.pathPredecessors;
        G.bestPathScore = N.bestPathScore;
        G.bestLeafID = N.bestLeafID;
        G.bestRootID = N.bestRootID;
        G = class(G,'graph');
    else
      	G.node_data = zeros(N,1);
        G.node_effectiveness = [];
        G.connectivity = sparse(1,1);
        G.edge_weights = spares(1,1);
        G.pathDistances = [];
        G.pathPredecessors = []; 
        G.shortestPaths = [];
    	G = class(G,'graph');
    end
else
	error('Unrecognized number of class constructor arguments');
end