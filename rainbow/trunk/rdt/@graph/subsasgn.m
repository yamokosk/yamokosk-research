function G = subsasgn(G,index,val)
% SUBSASGN Define index assignment for rrttree objects
switch index.type
case '()'
    switch index.subs{:}
    case 1
        G.v = val;
    case 2
        G.e = val;
    otherwise
        error('Index out of range')
    end
case '.'
    switch index.subs
    case 'v'
        G.v = val;
    case 'e'
        G.e = val;
    case 'v_ptr'
        G.v_ptr = val;
    case 'e_ptr'
        G.e_ptr = val;
    case 'BestPathScore'
        G.BestPathScore = val;
    case 'BestLeafId'
        G.BestLeafId = val;
    case 'BestRootId'
        G.BestRootId = val;
    otherwise
        error('Invalid field name')
    end
end