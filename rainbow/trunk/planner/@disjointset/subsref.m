function b = subsref(ds,s)
% SUBSREF 
switch s.type
case '()'
    ind = s.subs{:};
    b = ds.sets(ind);
case '{}'
    ind = s.subs{:};
    b = ds.sets{ind};
otherwise
   error('Specify value for x as p(x)')
end