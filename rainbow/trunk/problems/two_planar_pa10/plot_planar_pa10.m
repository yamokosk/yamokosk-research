function fig = plot_planar_pa10(q, rs, h)

fig = [];
if (nargin < 3)
    h = [];
end

if (isempty(h))
    fig = gcf;
    if ( ~strcmp('planar_plot', get(fig, 'UserData')) )
        close(fig);
        fig = createWindow();
        xlabel('--X--');
        ylabel('--Y--');
        zlabel('--Z--');
        grid on;
    end
else
    fig = h;
end

%[T_w_t, T_w_1, T_w_2, T_w_3] = fkine_planar_pa10(q, rs);
T_w_j = zeros(4,4,6);
for n = 1:6
    T_w_j(:,:,n) = kine(rs, q, n);
end

xcoords = zeros(2,5); ycoords = zeros(2,5); zcoords = zeros(2,5);
for n = 2:6
    xcoords(:,n-1) = [T_w_j(1,4,n-1); T_w_j(1,4,n)];
    ycoords(:,n-1) = [T_w_j(2,4,n-1); T_w_j(2,4,n)];
    zcoords(:,n-1) = [T_w_j(3,4,n-1); T_w_j(3,4,n)];
end
% xcoords = [T_w_1(1,4), T_w_2(1,4), T_w_3(1,4); ...
%            T_w_2(1,4), T_w_3(1,4), T_w_t(1,4)];
% ycoords = [T_w_1(2,4), T_w_2(2,4), T_w_3(2,4); ...
%            T_w_2(2,4), T_w_3(2,4), T_w_t(2,4)];
colors = ['k','r','g','b','k','r'];
nameprefix = {'link_1_', 'link_2_', 'link_3_', 'link_4_', 'link_5_', 'link_6_'};

% Get existing line segments.. if they exist
if (~isempty(rs.ghandles))
    % just update vertex data
	for n = 1:length(rs.ghandles)
        set(links(n), 'XData', xcoords(:,n));
        set(links(n), 'YData', ycoords(:,n));
        set(links(n), 'ZData', zcoords(:,n));
    end
else
    rs.ghandles = line(xcoords, ycoords, zcoords);
    
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