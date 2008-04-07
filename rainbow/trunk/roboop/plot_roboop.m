function h = plot_roboop(varargin)
% plot
%  
%   Overloaded plot function for RoboOp objects
%
%   Syntax
%
%       plot(robj)
%       plot(robj, q)
%       plot(robj1, q1, robj2, q2, ...)
%
%   Description
%
%   Some description

% Parse inputs
[robjs, qs] = parseInputs(varargin);

% Create a window if necessary
h = createWindow();

% Plot the robots
for n = 1:length(robjs)
    internal_plot(robjs(n), qs(:,n));
end


% Internal plot function
function h = internal_plot(robj, q)

T_w_j = zeros(4,4,6);
for n = 1:6
    T_w_j(:,:,n) = kine(robj, q, n);
end

xcoords = zeros(2,5); ycoords = zeros(2,5); zcoords = zeros(2,5);
for n = 2:6
    xcoords(:,n-1) = [T_w_j(1,4,n-1); T_w_j(1,4,n)];
    ycoords(:,n-1) = [T_w_j(2,4,n-1); T_w_j(2,4,n)];
    zcoords(:,n-1) = [T_w_j(3,4,n-1); T_w_j(3,4,n)];
end
colors = ['k','r','g','b','k','r'];
nameprefix = {'link_1_', 'link_2_', 'link_3_', 'link_4_', 'link_5_', 'link_6_'};

% Get existing line segments.. if they exist
ghandles = get(gca, 'UserData');

if ( isfield( ghandles, robj.name ) )
    lines = ghandles.(robj.name);
    % just update vertex data
    for n = 1:length(lines)
        set(lines(n), 'XData', xcoords(:,n));
        set(lines(n), 'YData', ycoords(:,n));
        set(lines(n), 'ZData', zcoords(:,n));
    end
else
    lines = line(xcoords, ycoords, zcoords);
    
    for n = 1:length(lines)
        set(lines(n), 'Color', colors(n));
        set(lines(n), 'LineWidth', 5);
        set(lines(n), 'UserData', [nameprefix{n}, robj.name]);
    end
    
    text(xcoords(1,1), ycoords(1,1), robj.name);
    ghandles.(robj.name) = lines;
end

set(gca, 'UserData', ghandles);


% Parse plot inputs
function [robjs, qs] = parseInputs(varargin)
input = varargin{1};
nargs = length(input);
qs = [];

k = 1;
for m = 1:2:nargs
    robjs(k) = input{m};
    
    if ( ~isstruct(robjs(k)) )
        error('Expected a RoboOp structure.');
    end
    
    if nargs >= m+1
        q = input{m+1};
        qs = [qs, q];
    else
        qs = [qs, zeros(robjs(k).dof,1)];
        for n = 1:robjs(k).dof
            qs(n,k) = robjs(k).links(n).q;
        end
    end
    k = k + 1;
end


% Create a window
function h = createWindow()

fh = get(0,'Children');     % Get handles to all figure windows
% Now loop through and check to see if a RoboOp figure window is open. If
% so, its a good idea to close it since problems can occur if we load a
% different scene description file.
oldFigNumber = -1;
for n = 1:length(fh)
    fig = fh(n);
    figUserData = get(fig, 'UserData');
    if ( ischar(figUserData) )
        if ( strcmp('RoboOp', figUserData) )
            oldFigNumber = fig;
            break;
        end
    end
end

if (oldFigNumber > 0)
	h = oldFigNumber;
else
    h = figure('UserData', 'RoboOp');
    
    % Scene axes
    set(gca, 'DataAspectRatio',   [1, 1, 1]);
    axis([-2 2 -1.133 1.133]);
    xlabel('--X--');
    ylabel('--Y--');
    zlabel('--Z--');
    grid on;
end

