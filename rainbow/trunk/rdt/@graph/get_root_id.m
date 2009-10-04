function root_id = get_root_id(G, child_id)
isRootId = G.RootIds(1,child_id);
root_id = child_id;

while ( ~isRootId )
    root_id = get_parent_node(G, root_id);
    isRootId = G.RootIds(1,root_id);
end