function ds = disjointset(a)
%DISJOINTSET Disjoint set class constructor.
%   ds = DISJOINTSET(v) creates a disjoint set from the vector v.
if nargin == 0
   ds.sets = [];
   ds = class(ds,'disjointset');
elseif isa(a,'disjointset')
   ds = a;
else
   ds.sets = sparse(1:a,1:a,1,a,a,a);
   ds = class(ds,'disjointset');
end