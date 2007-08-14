function NewSpaceID = dCreateHashSpace(SpaceID)
%CreateHashSpace   Creates a space object
%   CreateHashSpace(SpaceID) This space uses an internal data structure 
%   that records how each geom overlaps cells in one of several three 
%   dimensional grids. Each grid has cubical cells of side lengths 2i, 
%   where i is an integer that ranges from a minimum to a maximum value. 
%   The time required to do intersection testing for n objects is O(n) 
%   (as long as those objects are not clustered together too closely), 
%   as each object can be quickly paired with the objects around it.
%
%   Example
%      space = CreateHashSpace(0);
%
%   See also CreateSimpleSpace.

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

NewSpaceID = calllib('MODE','mHashSpaceCreate',SpaceID);