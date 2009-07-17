function b = subsref(G,index)
%SUBSREF Define field name indexing for rrttree objects
switch (index(1).type)
    case '.'
        switch (index(1).subs)
            case 'V'
                if ( G.NodeCount > 0 )
                    if ( length(index) == 1 )
                        b = G.NodeData(:, 1:G.NodeCount);
                    else
                        b = G.NodeData(:,index(2).subs{1});
                    end
                else
                    b = [];
                end
            case 'E'
                if ( G.NodeCount > 0 )
                    if ( length(index) == 1 )
                        b = G.Connectivity(1:G.NodeCount,1:G.NodeCount);
                    else
                        b = G.Connectivity(index(2).subs{1}, index(2).subs{1});
                    end
                else
                    b = [];
                end
            case 'Wv'
                if ( G.NodeCount > 0 )
                    if ( length(index) == 1 )
                        b = G.NodeWeights(1, 1:G.NodeCount);
                    else
                        b = G.NodeWeights(:,index(2).subs{1});
                    end
                else
                    b = [];
                end
            case 'We'
                if ( G.NodeCount > 0 )
                    if ( length(index) == 1 )
                        b = G.EdgeWeights(1:G.NodeCount, 1:G.NodeCount);
                    else
                        b = G.EdgeWeights(index(2).subs{1}, index(2).subs{1});
                    end
                else
                    b = [];
                end
            case 'Seff'
                if ( G.NodeCount > 0 )
                    if ( length(index) == 1 )
                        b = G.NodeEffectiveness(1,1:G.NodeCount);
                    else
                        b = G.NodeEffectiveness(index(2).subs{1},:);
                    end
                else
                    b = [];
                end
            case 'BestPathScore'
                b = G.BestPathScore;
            case 'BestLeafID'
                b = G.BestLeafId;
            case 'BestRootID'
                b = G.BestRootId;
            case 'NodeVisitedCount'
                b = G.NodeVisitedCount(1,1:G.NodeCount);
            case 'NodeExtendedCount'
                b = G.NodeExtendedCount(1,1:G.NodeCount);
            otherwise
                error('Invalid field name.');
        end
    otherwise
        error(['Operator ' index(1).type ' not supported by graph objects.']);
end