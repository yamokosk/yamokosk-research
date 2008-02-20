function G = graph(N)
% Generic graph class
%   G = graph;      Creates an empty graph.
%   G = graph(N)    Creates an empty graph where the nodes have degree N.
%   G2 = graph(G1)  Copies G1 into G2.
if nargin == 0
	G.node_data = [];
    G.edge_data = [];
    G.node_weights = [];
	G.connectivity = sparse(1,1);
	G = class(G,'graph');
elseif nargin == 1
    if ( isa(N,'graph') )
        G = N;
    else
      	G.node_data = zeros(N,1);
        G.connectivity = sparse(1,1);
        G.node_weights = [];
    	G = class(G,'graph');
    end
else
	error('Unrecognized number of class constructor arguments');
end