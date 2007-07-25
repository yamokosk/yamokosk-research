function initKDTree(ptcloud)
% INITKDTREE(PTCLOUD)  Initializes a KD-Tree data structure
%   INITKDTREE(PTCLOUD) builds a KD-tree data structure from the point 
%   cloud specified in PTCLOUD. PTCLOUD is interpreted as a (number of 
%   dimensions x number of points) array.
%
%   Example
%       A = rand(2,100);
%       initKDTree(A);
%
%   See also SEARCHKDTREE
mexSpatialSearch('Init',ptcloud);