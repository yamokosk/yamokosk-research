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
            case 'Vw'
                if ( length(index) == 1 )
                    b = G.node_weights;
                else
                    b = G.node_weights(:,index(2).subs{1});
                end
            case 'Ew'
                if ( length(index) == 1 )
                    b = G.edge_weights;
                else
                    b = G.edge_weights(index(2).subs{1}, index(2).subs{1});
                end
            case 'Seff'
                if ( length(index) == 1 )
                    b = G.node_effectiveness;
                else
                    b = G.node_effectiveness(index(2).subs{1},:);
                end
            case 'pathDist'
                if ( length(index) == 1 )
                    b = G.pathDistances;
                else
                    b = G.pathDistances(:,index(2).subs{1});
                end
            case 'pathPred'
                if ( length(index) == 1 )
                    b = G.pathPredecessors;
                else
                    b = G.pathPredecessors(:,index(2).subs{1});
                end                    
            otherwise
                error('Invalid field name.');
        end
    otherwise
        error(['Operator ' index(1).type ' not supported by graph objects.']);
end