function findOptimalRobotOffset(t, traj, xoff1, xoff2, z, isSourceRbt)

    pp1 = interp1(t,[traj(:,1)+xoff1, traj(:,2), traj(:,3)],'spline','pp');
    pp2 = interp1(t,[traj(:,1)+xoff2, traj(:,2), traj(:,3)],'spline','pp');

    if (isSourceRbt)
        x = linspace(xoff2,xoff2+1,100);
    else
        x = linspace(xoff2,xoff2-1,100);
    end
    
    dt1 = []; dt2 = [];
    for xi = x
        [t0,tf] = findSplineSphereIntersect(t, pp1, [xi;0;z], 1-0.07);
        dt1 = [dt1; tf-t0];
        [t0,tf] = findSplineSphereIntersect(t, pp2, [xi;0;z], 1-0.07);
        dt2 = [dt2; tf-t0];
    end
    
    if (isSourceRbt)
        figure(1)
    else
        figure(2)
    end
    plot(x,dt1,'k--',x,dt2,':');
    legend(['\delta_{min} = ' num2str(xoff1)], ['\delta_{max} = ' num2str(xoff2)])
    if (isSourceRbt)
        title('Source robot')
    else
        title('Sensor robot')
    end
%     x = lsqnonlin(@mycost, x_guess);
%     
%     function f = mycost(x)
%         [t1,tf] = findSplineSphereIntersect(t, pp1, [x;y;z], 1-0.07);
%         %dt1 = tf - t0;
%         
%         [t2,tf] = findSplineSphereIntersect(t, pp2, [x;y;z], 1-0.07);
%         %dt2 = tf - t0;
%         
%         f = [t1; t2; x];
%     end
% 
%     [t0,tf] = findSplineSphereIntersect(t, pp1, [x;y;z], 1-0.07)
%     [t0,tf] = findSplineSphereIntersect(t, pp2, [x;y;z], 1-0.07)
end