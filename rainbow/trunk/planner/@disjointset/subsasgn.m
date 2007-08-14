function ds = subsasgn(ds,index,val)
% SUBSASGN Define index assignment for asset objects
switch index.type
case '{}'
   ind = index.subs{:};
   ds.sets{ind} = val;
end