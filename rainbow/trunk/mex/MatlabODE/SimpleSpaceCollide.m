function inCollision = SimpleSpaceCollide(SpaceID)
%SimpleSpaceCollide   Checks for collision inside a space
%   SimpleSpaceCollide(SpaceID) checks for collisions inside a space and 
%   simply returns a TRUE or FALSE value if a collision is present or not.
%
%   Example
%      InitMODE
%      space = CreateHashSpace(0);
%      sphere2 = CreateSphere(space,2);
%      sphere3 = CreateSphere(space,3);
%      SetGeomPosition(sphere2,[3,0,0]);
%      bInCollision = SimpleSpaceCollide(space)
%
%   See also SimpleCollide, Collide, SimpleSpaceCollide2

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
%   ODE is Copyright � 2001-2004 Russell L. Smith. All rights reserved.

if ~libisloaded('MODE')
	error('Collision detection library is not currently loaded!');
end

inCollision = calllib('MODE','mSimpleSpaceCollide',SpaceID);