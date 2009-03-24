function [t0,tf] = findSplineSphereIntersect(tv, pp, c, r)

t0 = fminsearch(@myfun,tv(1));
tf = fminsearch(@myfun,tv(end));

    function f = myfun(t)
        x = ppval(pp,t);
        d = x - c;
        f = abs(d'*d - r^2);
    end

% data = ppval(pp,tv);
% figure();
% plot3(data(:,1),data(:,2),data(:,3))
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% hold on; 
% h = drawSphere(c,r);
% set(h, 'FaceAlpha', 0.1, 'FaceColor', [0.2, 0.5, 0.2]);
% 
% x0 = ppval(pp,t0);
% plot3(x0(1),x0(2),x0(3),'*');
% f = myfun(t0)
% xf = ppval(pp,tf);
% plot3(xf(1),xf(2),xf(3),'*');

end