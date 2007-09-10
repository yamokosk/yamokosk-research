function ds = union_set(ds,to,from)
% Inputs
%   ds - class object
%   c,n - two objects we will place in the same set
% pull out data we are going to combine
tosetID = find_ind(ds,to);
[fromset,fromsetID] = find_set(ds,from);

% Move b,indb into a,inda
ds.sets(fromset, tosetID) = 1;
ds.sets(fromset, fromsetID) = 0;
end % END UNION_SET