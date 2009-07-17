function test = test_graph

test = load_tests_from_mfile(test_loader);

    function set_up
    end
    
    function tear_down
    end

    function test_add_node
        G = graph;
        nodes = [1 2 1 3 9 1];
        eff = rand(1,6);
        [G,ind] = add_node(G,nodes,eff,true);
        assert(length(nodes) == length(G.V));
        
        [G,ind] = add_node(G,nodes,eff,false);
        assert(length(nodes)*2 == length(G.V));
    end
end