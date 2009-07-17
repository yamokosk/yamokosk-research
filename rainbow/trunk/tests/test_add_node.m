function test_add_node()

G = graph;

nodes = [1 2 1 3 9 1];

eff = rand(1,6);

[G,ind] = add_node(G,nodes,eff,true);

