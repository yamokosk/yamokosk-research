function [pos, normal, depth] = Collide2(ID1, ID2, NumContactPts)
%COLLIDE2   Check for collision between to geometry objects
%   COLLIDE2(ID1,ID2, NUMCONTACTPTS) checks for collision between two ODE 
%   geometry objects specified by ID1 and ID2. It returns the position, 
%   normal vector and depth of the contact points if they exist. The
%   desired number of contact points is controlled by the NUMCONTACTPTS 
%   variable.
%
%   Example
%       InitMODE
%       space = CreateHashSpace(0);
%       sphere2 = CreateSphere(space,2);
%       sphere3 = CreateSphere(space,3);
%       SetGeomPosition(sphere2,[3,0,0]);
%       [p,n,d] = Collide2(sphere2,sphere3,2)
%
%   See also COLLIDE.

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

pContactGeom = calllib('MODE','mCollide2',ID1,ID2,NumContactPts);
sContactGeom = get(pContactGeom, 'Value');
pos = sContactGeom.pos;
normal = sContactGeom.normal;
depth = sContactGeom.depth;
calllib('MODE','mDeallocateGeomContact',pContactGeom);