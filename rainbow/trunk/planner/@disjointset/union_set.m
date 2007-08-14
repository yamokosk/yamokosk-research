function ds = union_set(ds,c,n)
% pull out data we are going to combine
[a,inda] = find_set(ds,c);
[b,indb] = find_set(ds,n);
len = length(ds);
otherind = setxor(1:len,[inda,indb]);
othersets = ds.sets(otherind);

% create new set
newset = union(a,b);

% redefine ds struct
ds.sets = othersets;
ds.sets{end+1} = newset;

end % END UNION_SET