function robj = roboop(conffile, robotname)

if ( ~ischar( conffile ) && ~ischar( robotname ) )                                             
    error('Arguments must be strings!');
end

fullconfname = fullfile(cd, conffile);

robj = mex_roboop(fullconfname, robotname);