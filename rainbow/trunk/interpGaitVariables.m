function dyn = interpGaitVariables(pp, t);
x = ppval(pp,t)';
xc = cell(12,1);
ptr = 1;
for n = 1:12
    %xc(n) = {x(1:3),x(4:6),x(7:9),x(10:12),x(13:15),x(16:18),x(19:21),x(22:24),x(25:27),x(28:};
    xc{n} = x(ptr:ptr+2);
    ptr = ptr + 3;
end
vnames = {'pelvis_trans'; 'pelvis_rot'; 'lhip'; 'lknee'; 'lankle'; 'lshoulder'; 'lelbow'; 'rhip'; 'rknee'; 'rankle'; 'rshoulder'; 'relbow'};

dyn = [vnames, xc];
