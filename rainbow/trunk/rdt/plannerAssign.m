function Prob = plannerAssign(varargin)
Prob = [];

if (nargin == 1)
    Prob = plannerAssignFromDir(varargin{1});
else
    Prob = plannerAssignFromArgs(varargin{:});
end