function val = same_connected_component(ds,c,n)
% Inputs
%   ds - class object
%   c,n  - indices to test whether they are in same component
if isempty( setxor(find_set(ds,c), find_set(ds,n)) )
    val = true;
else
    val = false;
end