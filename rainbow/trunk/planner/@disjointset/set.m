function ds = set(ds,varargin)
% SET Set asset properties and return the updated object
propertyArgIn = varargin;
while length(propertyArgIn) >= 2,
   prop = propertyArgIn{1};
   val = propertyArgIn{2};
   propertyArgIn = propertyArgIn(3:end);
   switch prop
   case 'Sets'
      ds.sets = val;
   otherwise
      error('Disjoint set properties: Sets')
   end
end