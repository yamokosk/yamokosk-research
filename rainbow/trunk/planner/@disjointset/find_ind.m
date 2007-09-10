function ind = find_ind(ds,c)
% Inputs:
%   ds - class object
%   c  - index to find
% Outputs:
%   ind - set index c belongs to
ind = find(ds.sets(c,:) ~= 0);