function val = same_connected_component(ds,c,n)

if isempty( setxor(find_set(ds,c), find_set(ds,n)) )
    val = true;
else
    val = false;
end