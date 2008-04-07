function robj = roboop(conffile, robotname, T_f_base, T_EE_tool)
%   Syntax
%
%       robj = roboop(conffile, robotname)
%       robj = roboop(conffile, robotname, T_f_base)
%       robj = roboop(conffile, robotname, T_f_base, T_EE_tool)
%
%   Description
%
%   Creates a RoboOp robot object from the robot parameters specified in
%   the *.conf file. One may also specify custom transformations between
%   the world coordinate system and the base, T_f_base, and between the
%   robot end-effector and a tool coordinate system, T_EE_tool.
if ( nargin < 4 )
    T_EE_tool = eye(4);
    if ( nargin < 3 )
        T_f_base = eye(4);
    end
end

if ( ~ischar( conffile ) && ~ischar( robotname ) )                                             
    error('Arguments must be strings!');
end

fid = fopen(conffile);
if (fid < 0)
    error([conffile ' does not seem to exist. Please check the path and filename.']);
end
fclose(fid);

robj = mex_roboop(conffile, robotname);
robj.T_f_base = T_f_base;
robj.T_EE_tool = T_EE_tool;