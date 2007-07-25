function ID = CreateSphere(SpaceID, Radius)
%CREATESPHERE   Create a sphere geometry object
%   CREATESPHERE(SPACEID,RADIUS) creates a sphere geometry object inside
%   the space defined by SPACEID with a radius defined by RADIUS.
%   CREATESPHERE returns the ID of the newly created geometry object.
%
%   Example
%      space = CreateSimpleSpace(0);
%      sphere = CreateSphere(space, 5.0);
%
%   See also .

%   MODE (MATLAB interface to ODE) is Copyright (C) 2007 John Yamokoski
% 
%   This library is free software; you can redistribute it and/or
%   modify it under the terms of the GNU Lesser General Public
%   License as published by the Free Software Foundation; either
%   version 2.1 of the License, or (at your option) any later version.
% 
%   This library is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
%   Lesser General Public License for more details.
% 
%   You should have received a copy of the GNU Lesser General Public
%   License along with this library; if not, write to the Free Software
%   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
%
%   ODE is Copyright © 2001-2004 Russell L. Smith. All rights reserved.

if ~libisloaded('MODE')
	error('Collision detection library is not currently loaded!');
end

ID = calllib('MODE','mCreateSphere',SpaceID, Radius);
