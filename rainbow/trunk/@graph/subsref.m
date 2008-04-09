function b = subsref(G,index)
%SUBSREF Define field name indexing for rrttree objects
switch (index(1).type)
    case '.'
        switch (index(1).subs)
            case 'V'
                if ( length(index) == 1 )
                    b = G.node_data;
                else
                    b = G.node_data(:,index(2).subs{1});
                end
            case 'E'
                if ( length(index) == 1 )
                    b = G.connectivity;
                else
                    b = G.connectivity(index(2).subs{1}, index(2).subs{1});
                end
            case 'Wv'
                if ( length(index) == 1 )
                    b = G.node_weights;
                else
                    b = G.node_weights(index(2).subs{1});
                end
            otherwise
                error('Invalid field name.');
        end
    otherwise
        error(['Operator ' index(1).type ' not supported by graph objects.']);
end