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


MODE_API int mCreateCCylinder (int SID, double r, double len)
{
	// Locate the ODE space object from its ID
	dSpaceID dSID = NULL;
	if (SID != 0) mCHECKINT(gObjManager.get(SID,dSID), "Specified SpaceID does not exist.");
	
	// Call ODE library
	dGeomID dGID = dCreateCCylinder (dSID, r,len);

	// Store object in manager
	return gObjManager.add(dGID);
}

MODE_API void mGeomCCylinderSetParams (int GID, double r, double len)
{
	dGeomID dGID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	dGeomCCylinderSetParams(dGID, r,len);
}

MODE_API mxArray* mGeomCCylinderGetParams (int GID)
{
	dGeomID dGID = NULL;
	mCHECKNULL(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");

	dVector3 result = {0};
	dGeomCCylinderGetParams(dGID, result, (result + 1));
	
	mxArray* v = mxCreateDoubleMatrix(2,1,mxREAL);
	double* pv = mxGetPr(v);
	for (int n=0; n < 2; ++n) pv[n] = result[n];

	return v;
}

MODE_API double mGeomCCylinderPointDepth (int GID, double x, double y, double z)
{
	dGeomID dGID = NULL;
	mCHECKINT(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	return dGeomCCylinderPointDepth(dGID, x, y, z);
}


#ifdef __cplusplus
}
#endif