function [cc,ind] = find_set(ds,c)
% Inputs:
%   ds - class object
%   c  - index to find
% Outputs:
%   cc - connected component c belongs to
%   ind - set index c belongs to
ind = find(ds.sets(c,:) ~= 0);
cc = find(ds.sets(:,ind) ~= 0);