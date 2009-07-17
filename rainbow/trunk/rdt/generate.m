% =========================================================================
% generate(Tj, N, ngen)
%
%   Generate a set of nodes for target Tj. Employs the user's  generation 
%   function to do all the heavy lifting.
% =========================================================================
function Vj = generate(t_step, Tbar, N, ngen)
Vj = [];
for n = 1:N
    for m = 1:N
        Vtemp = ngen(t_step, Tbar);
        if ( ~isempty(Vtemp) )
            Vj = [Vj, Vtemp];
            break;
        end
    end
end