function fig = plot_planar_pa10(q,h)

if (isempty(h))
    fig = figure();
else
    fig = h;
    figure(fig);
end

[T_0_3, T_0_1, T_0_2] = fkine_planar_pa10(q);

j1 = T_0_1(1:2,4);
j2 = T_0_2(1:2,4);
j3 = T_0_3(1:2,4);

% Draw circles at joint centers
plot([j1(1); j2(1); j3(1)], [j1(2); j2(2); j3(2)],'o');

% Link 0-1
line([0; j1(1)], [0; j1(2)]);

% Link 1-2
line([j1(1); j2(1)], [j1(2); j2(2)]);

% Link 2-3
line([j2(1); j3(1)], [j2(2); j3(2)]);

% Tool
%line(x3,y2);