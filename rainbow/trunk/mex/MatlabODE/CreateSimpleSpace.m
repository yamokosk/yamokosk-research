function NewSpaceID = CreateSimpleSpace(SpaceID)
%CreateSimpleSpace   Creates a space object
%   CreateSimpleSpace(SpaceID) creates a space which does not do any 
%   collision culling - it simply checks every possible pair of geoms for 
%   intersection, and reports the pairs whose AABBs overlap. The time 
%   required to do intersection testing for n objects is O(n^2). This 
%   should not be used for large numbers of objects, but it can be the 
%   preferred algorithm for a small number of objects. This is also 
%   useful for debugging potential problems with the collision system. If
%   SpaceID is nonzero, insert the new space into that space.
%
%   Example
%      space = CreateSimpleSpace(0);
%
%   See also CreateHashSpace.

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

NewSpaceID = calllib('MODE','mSimpleSpaceCreate',SpaceID);