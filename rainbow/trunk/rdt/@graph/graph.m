function G = graph(varargin)
% Generic graph class
%   G = graph;      Creates an empty graph.
%   G = graph(N)    Creates an empty graph where the nodes have degree N.
%   G2 = graph(G1)  Copies G1 into G2.

members = { 'RootIds', ...
            'LeafIds', ...
            'NodeData', ...
            'NodeWeights', ...
            'NodeEffectiveness', ...
            'EdgeWeights', ...
            'ShortestPathsInvalid', ...
            'ShortestPathDistances', ...
            'ShortestPathPredecessors', ...
            'BestPathScore', ...
            'BestLeafId', ...
            'BestRootId', ...
            'NodeVisitedCount', ...
            'NodeExtendedCount', ...
            'NodeCount', ...
            'MemSize'};

for n = 1:size(members,2)
    name = members{1,n};
    G.(name) = [];
end
PreAllocation = 500;
G = setfield(G, 'RootIds', sparse(1,PreAllocation));
G = setfield(G, 'LeafIds', sparse(1,PreAllocation));
G = setfield(G, 'NodeData', zeros(1,PreAllocation));
G = setfield(G, 'NodeWeights', zeros(1,PreAllocation));
G = setfield(G, 'NodeEffectiveness', zeros(1,PreAllocation));
G = setfield(G, 'EdgeWeights', sparse(PreAllocation,PreAllocation));
G = setfield(G, 'ShortestPathsInvalid', true);
G = setfield(G, 'BestPathScore', 0);
G = setfield(G, 'BestLeafId', 0);
G = setfield(G, 'BestRootId', 0);
G = setfield(G, 'NodeVisitedCount', sparse(1, PreAllocation));
G = setfield(G, 'NodeExtendedCount', sparse(1, PreAllocation));
G = setfield(G, 'NodeCount', 0);
G = setfield(G, 'MemSize', PreAllocation);

if (nargin == 1)
    argin = varargin{1};
    
    if ( isa(argin, 'graph') )
        for n = 1:size(members,2)
            name = members{1,n};
            G.(name) = argin.(name);
        end
    elseif ( isa(argin, 'struct') )
        for n = 1:size(members,2)
            name = members{1,n};
            if ( isfield(argin, name) )
                G.(name) = argin.(name);
            end
        end
    elseif ( isa(argin, 'numeric') )
        G = setfield(G, 'NodeData', zeros(argin,PreAllocation));
    else
        error('Argument #1 must be either a graph, struct, or numeric value.'); 
    end
end

G = class(G,'graph');