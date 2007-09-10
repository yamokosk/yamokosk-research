function [d pred rank] = generatePath(S,G,s)

[d pred rank]=astar_search(G,21,@dist);
    
    function h = dist(u)
        h = 1.9768 - S(u,13);
    end

end