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

using namespace std;

typedef vector< pair<int,int> > PairContainer;

#ifdef __cplusplus
extern "C" {
#endif

MODE_API int mCreateGeomTransform (int ID)
{
	dSpaceID dID = NULL;
	if (ID != 0) mCHECKINT(gObjManager.get(ID,dID), "Specified SpaceID does not exist.");
	return gObjManager.add( dCreateGeomTransform(dID) );
}

MODE_API void mGeomTransformSetGeom (int g, int obj)
{
	dGeomID dg = NULL, dobj = NULL;
	mCHECKVOID(gObjManager.get(g,dg), "Specified GeomID does not exist.");
	mCHECKVOID(gObjManager.get(obj,dobj), "Specified GeomID does not exist.");

	dGeomTransformSetGeom(dg,dobj);
}

MODE_API int mGeomTransformGetGeom (int ID)
{
	dGeomID dID = NULL;
	mCHECKINT(gObjManager.get(ID,dID), "Specified GeomID does not exist.");

	dID = dGeomTransformGetGeom(dID);
	mCHECKINT(gObjManager.id(ID,dID), "Could not locate ID by ODE object pointer.");
	return ID;
}

MODE_API void mGeomTransformSetCleanup (int ID, int mode)
{
	dGeomID dID = NULL;
	mCHECKVOID(gObjManager.get(ID,dID), "Specified GeomID does not exist.");

	dGeomTransformSetCleanup(dID, mode);
}

MODE_API int mGeomTransformGetCleanup (int ID)
{
	dGeomID dID = NULL;
	mCHECKINT(gObjManager.get(ID,dID), "Specified GeomID does not exist.");

	return dGeomTransformGetCleanup(dID);
}

MODE_API void mGeomTransformSetInfo (int ID, int mode)
{
	dGeomID dID = NULL;
	mCHECKVOID(gObjManager.get(ID,dID), "Specified GeomID does not exist.");

	dGeomTransformSetInfo(dID, mode);
}

MODE_API int mGeomTransformGetInfo (int ID)
{
	dGeomID dID = NULL;
	mCHECKINT(gObjManager.get(ID,dID), "Specified GeomID does not exist.");

	return dGeomTransformGetInfo(dID);
}


#ifdef __cplusplus
}
#endif