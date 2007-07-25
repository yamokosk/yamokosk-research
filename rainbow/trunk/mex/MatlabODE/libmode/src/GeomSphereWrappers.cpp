//   MODE (MATLAB interface to ODE) is Copyright (C) 2007 John Yamokoski
// 
//   This library is free software; you can redistribute it and/or
//   modify it under the terms of the GNU Lesser General Public
//   License as published by the Free Software Foundation; either
//   version 2.1 of the License, or (at your option) any later version.
// 
//   This library is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//  Lesser General Public License for more details.
// 
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
//  ODE is Copyright © 2001-2004 Russell L. Smith. All rights reserved.

#include "ODEMatlabAPI.h"
#include "ODEManager.h"

extern ODEManager gObjManager;

#ifdef __cplusplus
extern "C" {
#endif

MODE_API int mCreateSphere(int SID, double radius)
{
	// Locate the ODE space object from its ID
	dSpaceID dSID = NULL;
	mxAssert(gObjManager.get(SID,dSID), "Specified SpaceID does not exist.");
	
	// Call ODE library
	dGeomID dGID = dCreateSphere (dSID, radius);

	// Store object in manager
	return gObjManager.add(dGID);
}

MODE_API void mGeomSphereSetRadius(int GID, double radius)
{
	dGeomID dGID = NULL;
	mxAssert(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	dGeomSphereSetRadius(dGID, radius);
}

MODE_API double mGeomSphereGetRadius(int ID)
{
	dGeomID dGID = NULL;
	mxAssert(gObjManager.get(ID,dGID), "Specified GeomID does not exist.");
	return dGeomSphereGetRadius(dGID);
}

MODE_API double mGeomSpherePointDepth(int ID, double x, double y, double z)
{
	dGeomID dGID = NULL;
	mxAssert(gObjManager.get(ID,dGID), "Specified GeomID does not exist.");
	return dGeomSpherePointDepth(dGID, x, y, z);
}

#ifdef __cplusplus
}
#endif

