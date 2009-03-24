function varargout = drawSphere(varargin)
%DRAWSPHERE draw a sphere as a mesh
%
%   drawSphere(XC, YC, ZC, R)
%   drawSphere([XC YC ZC], R)
%   drawSphere([XC YC ZC R])
%
%
%   ---------
%
%   author : David Legland 
%   INRA - TPV URPOI - BIA IMASTE
%   created the 17/02/2005
%

%   HISTORY

options = {};
for i=1:length(varargin)
    if ischar(varargin{i})        
        options = varargin(i:end);
        varargin = varargin(1:i-1);
        break;
    end
end


if length(varargin)==1
    sphere = varargin{1};
    xc = sphere(:,1);
    yc = sphere(:,2);
    zc = sphere(:,3);
    r  = sphere(:,4);
elseif length(varargin)==2
    center = varargin{1};
    xc = center(1);
    yc = center(2);
    zc = center(3);
    r  = varargin{2};
elseif length(varargin)==4
    xc = varargin{1};
    yc = varargin{2};
    zc = varargin{3};
    r  = varargin{4};
else
    error('drawSphere : please specify center and radius');
end



nphi = 32;
ntheta = 16;


theta = (0:ntheta)/ntheta*pi;
%theta = (0:ntheta)/ntheta*pi/2;
phi = (0:nphi)/nphi*2*pi;
%phi = (0:nphi)/nphi*pi;

sintheta = sin(theta);

x = xc + cos(phi')*sintheta*r;
y = yc + sin(phi')*sintheta*r;
z = zc + ones(length(phi),1)*cos(theta)*r;


if nargout == 0
    %surf(x,y,z, 'FaceColor', 'g', options{:});
    surf(x,y,z, options{:});
elseif nargout == 1
    %varargout{1} = surf(x,y,z, 'FaceColor', 'g', options{:});
    varargout{1} = surf(x,y,z, options{:});
elseif nargout == 3
    varargou{1} = x; 
    varargou{1} = y; 
    varargou{1} = z; 
end

