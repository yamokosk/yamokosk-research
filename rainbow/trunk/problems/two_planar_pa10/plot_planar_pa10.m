function fig = plot_planar_pa10(q, rs, h)

fig = [];
if (isempty(h))
    fig = gcf;
    if ( ~strcmp('planar_plot', get(fig, 'UserData')) )
        close(fig);
        fig = createWindow();
    end
else
    fig = h;
end

[T_w_t, T_w_1, T_w_2, T_w_3] = fkine_planar_pa10(q, rs);

xcoords = [T_w_1(1,4), T_w_2(1,4), T_w_3(1,4); ...
           T_w_2(1,4), T_w_3(1,4), T_w_t(1,4)];
ycoords = [T_w_1(2,4), T_w_2(2,4), T_w_3(2,4); ...
           T_w_2(2,4), T_w_3(2,4), T_w_t(2,4)];
colors = ['k','r','g','b'];
nameprefix = {'link_1_', 'link_2_', 'link_3_', 'link_4_'};

% Get existing line segments.. if they exist
if (~isempty(rs.ghandles))
    % just update vertex data
	for n = 1:length(rs.ghandles)
        set(links(n), 'XData', xcoords(:,n));
        set(links(n), 'YData', ycoords(:,n));
    end
else
    rs.ghandles = line(xcoords, ycoords);
    
    for n = 1:length(rs.ghandles)
        set(rs.ghandles(n), 'Color', colors(n));
        set(rs.ghandles(n), 'LineWidth', 5);
        set(rs.ghandles(n), 'UserData', [nameprefix{n}, rs.name]);
    end
end



function fig = createWindow()
fig = figure('UserData', 'planar_plot');

% Scene axes
set(gca, 'DataAspectRatio',   [1, 1, 1]);
axis([-2 2 -1.133 1.133]);