function F = animateScene(scene, pp, t)
NF = length(t);
q = [{'src1',0.0};{'src2',pi/4};{'src3',pi/2};{'src4',0};{'src5',-pi/4};{'src6',0};{'sen1',0.0};{'sen2',pi/4};{'sen3',pi/2};{'sen4',0};{'sen5',-pi/4};{'sen6',0}];
g = interpGaitVariables(pp,0);
%scene = buildScene(scene,[q; g]);
%scene = drawScene(scene);
%drawnow;
%rect = getrect(scene.disp.fig);
%mov = avifile('example.avi');

for n = 1:NF
    g = interpGaitVariables(pp,t(n));
    scene = buildScene(scene,[q; g]);
    scene = drawScene(scene);
    %drawnow;
    %mov(n) = getframe(scene.disp.fig,rect);
    F(n) = getframe(scene.disp.GLAxes);
    %mov = addframe(mov,F);    
end
%mov = close(mov);