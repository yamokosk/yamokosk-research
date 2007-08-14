function h = drawBox(T, sides, varargin)


if size(varargin) == 1
    % Use existing handle
    h = varargin{1};
    set(h, 'Vertices', v');
else
    % Create a new patch object
    f = [1, 2, 6, 5; ...
         2, 3, 7, 6; ...
         3, 4, 8, 7; ...
         4, 1, 5, 8; ...
         1, 2, 3, 4; ...
         5, 6, 7, 8];
    
    h = patch('Vertices', v', 'Faces', f, 'FaceColor', 'r');
end