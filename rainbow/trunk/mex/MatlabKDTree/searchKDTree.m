function [nn,d,ind] = searchKDTree(pt,varargin)
% SEARCHKDTREE  Queries a KD-Tree data structure
%   NN = SEARCHKDTREE(PT) returns the nearest neighbor for point PT in a
%   KD-Tree. 
%   
%   NN = SEARCHKDTREE(PT,N) performs an N-nearest neighbor query for point 
%   PT.
%
%   NN = SEARCHKDTREE(PT,N,FUNCS) performs an N-nearest neighbor query for 
%   point PT and utilizes four user defined distance functions contained
%   in the structure FUNCS.
%
%       funcs.transformed_distance(q,r)
%           Returns the transformed distance between q and r.
%       funcs.max_distance_to_rectangle(p,lb,ub)
%       funcs.min_distance_to_rectangle(p,lb,ub)
%           Returns the transformed distance between p and the point on
%           the boundary of r farthest and closest to p, respectively. 
%           The boundary of r is defined by its lower most boundary (lb) 
%           and upper most boundary (ub) points.
%           points.
%       funcs.new_distance(dist, old, new, cutting_dimension)
%           Updates dist incrementally and returns the updated distance.
%
%   [NN,D] = SEARCHKDTREE(PT,...) returns the associated distances with 
%   each neighbor.
%
%   [NN,D,IND] = SEARCHKDTREE(PT,...) returns the column indicies to the
%   original Matlab array used to populate the search tree.
%
%   Example
%       A = rand(2,100);
%       initKDTree(A);
%       [nn,d] = searchKDTree([.5;.5],4)
%
%   See also INITKDTREE DEFAULTDISTFUNCS
funcs = [];
n = [];

switch (nargin)
    case 1
        % Do something
        n = 1;
        funcs = defaultDistFuncs();
    case 2
        n = varargin{1};
        funcs = defaultDistFuncs();
    case 3
        n = varargin{1};
        funcs = varargin{2};
    otherwise
        error('searchKDTree only accepts three arguments.');
end
        
[nn,d,ind] = mexSpatialSearch('Search',n,pt,funcs);