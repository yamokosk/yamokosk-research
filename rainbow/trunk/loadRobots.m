function [src, sen] = loadRobots()

src = loadPA10Params(['_src']);
src.base = transl(1.1398, 0 ,.601345) * rotz(pi);
src.tool = rotx(pi);
sen = loadPA10Params(['_sen']);
sen.base = transl(-1.1398, 0, .601345);
sen.tool = eye(4);