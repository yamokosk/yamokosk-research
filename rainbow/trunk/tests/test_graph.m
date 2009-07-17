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

    function test_add_edge
        G = graph;
        nodes = [1 2 1 3 9 1];
        eff = rand(1,6);
        [G,ind] = add_node(G,nodes,eff,true);
        
        for n = 2:length(ind)
            G = add_edge(G,ind(n-1),ind(n),n/10);
        end
        
        conn = get_connectivity(G);
        desired_conn = [0, 1, 0, 0, 0, 0; ...
                        0, 0, 1, 0, 0, 0; ...
                        0, 0, 0, 1, 0, 0; ...
                        0, 0, 0, 0, 1, 0; ...
                        0, 0, 0, 0, 0, 1; ...
                        0, 0, 0, 0, 0, 0];
        desired_conn = sparse(desired_conn);
        test = (conn == desired_conn);
        [r,c] = size(test);
        for r = 1:size(test,1)
            for c = 1:size(test,2)
                assert( test(r,c) );
            end
        end
    end

    function test_resize_internal_storage
        G = graph;
        nodes = [1 2 1 3 9 1];
        eff = rand(1,6);
        [G,ind] = add_node(G,nodes,eff,true);
        
        assert(length(nodes) == length(G.V));
        
        for n = 2:length(ind)
            G = add_edge(G,ind(n-1),ind(n),n/10);
        end
        conn = get_connectivity(G);
        
        desired_conn = [0, 1, 0, 0, 0, 0; ...
                        0, 0, 1, 0, 0, 0; ...
                        0, 0, 0, 1, 0, 0; ...
                        0, 0, 0, 0, 1, 0; ...
                        0, 0, 0, 0, 0, 1; ...
                        0, 0, 0, 0, 0, 0];
        desired_conn = sparse(desired_conn);
        test = (conn == desired_conn);
        [r,c] = size(test);
        for r = 1:size(test,1)
            for c = 1:size(test,2)
                assert( test(r,c) );
            end
        end
        
        % resize the internal graph storage
        G = resize_internal_storage(G,100,1);
        
        % check we still have same number of nodes
        assert(length(nodes) == length(G.V));
        
        % and that the connectivity info did not change
        conn = get_connectivity(G);
        test = (conn == desired_conn);
        [r,c] = size(test);
        for r = 1:size(test,1)
            for c = 1:size(test,2)
                assert( test(r,c) );
            end
        end
    end

    function test_compute_shortest_paths
        G = graph;
        nodes = [1 2 1 3 9 1];
        eff = rand(1,6);
        ind = zeros(size(nodes));
        
        % Add one root node
        [G,ind(1)] = add_node(G,nodes(1),eff(1),true);
        
        % Add the rest as non-root nodes
        [G,ind(2:end)] = add_node(G,nodes(2:end),eff(2:end),false);
        
        % Connect them
        for n = 2:length(ind)
            G = add_edge(G,ind(n-1),ind(n),n/10);
        end
        
        % Compute the shortest paths
        G = compute_shortest_paths(G);
    end

    function test_get_leaf_ids
        G = graph;
        nodes = [1 2 1 3 9 1];
        eff = rand(1,6);
        ind = zeros(size(nodes));
        
        % Add one root node
        [G,ind(1)] = add_node(G,nodes(1),eff(1),true);
        
        % Add the rest as non-root nodes
        [G,ind(2:end)] = add_node(G,nodes(2:end),eff(2:end),false);
        
        % Connect them
        for n = 2:length(ind)
            G = add_edge(G,ind(n-1),ind(n),n/10);
        end
        
        leafids = get_leaf_ids(G);
        assert( leafids == ind(end) );
    end

    function test_get_root_id
        G = graph;
        nodes = [1 2 1 3 9 1];
        eff = rand(1,6);
        ind = zeros(size(nodes));
        
        % Add one root node
        [G,ind(1)] = add_node(G,nodes(1),eff(1),true);
        
        % Add the rest as non-root nodes
        [G,ind(2:end)] = add_node(G,nodes(2:end),eff(2:end),false);
        
        % Connect them
        for n = 2:length(ind)
            G = add_edge(G,ind(n-1),ind(n),n/10);
        end
        
        root_id = get_root_id(G, ind(end));
        
        assert(root_id == ind(1));
    end

    function test_add_branch
        G = graph;
        nodes = [1 2 1 3 9 1];
        eff = rand(1,6);
        weights = rand(1,6);
        ind = zeros(size(nodes));
        
        % Add one root node
        [G,ind(1)] = add_node(G,nodes(1),eff(1),true);
        G = compute_shortest_paths(G);
        
        % Use add branch to add the rest of the nodes
        [G, ind(2:end)] = add_branch(G, ind(1), nodes(2:end), weights, eff(2:end));
    end
end