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

MODE_API int mSimpleSpaceCreate(int ID)
{
	dSpaceID dID = NULL;
	if (ID != 0) mCHECKINT(gObjManager.get(ID,dID), "Specified SpaceID does not exist.");
	return gObjManager.add(dSimpleSpaceCreate(dID));
}

MODE_API int mHashSpaceCreate(int ID)
{
	dSpaceID dID = NULL;
	if (ID !=0) mCHECKINT(gObjManager.get(ID,dID), "Specified SpaceID does not exist.");
	return gObjManager.add(dHashSpaceCreate(dID));
}

MODE_API int mQuadTreeSpaceCreate(int ID, const mxArray* Center, const mxArray* Extents, int Depth)
{
	dSpaceID dID = NULL;
	if (ID != 0) mCHECKINT(gObjManager.get(ID,dID), "Specified SpaceID does not exist.");

	dVector3 dCenter, dExtents;
	MEMCPY(dCenter,mxGetPr(Center),3);
	MEMCPY(dExtents,mxGetPr(Extents),3);
	return gObjManager.add(dQuadTreeSpaceCreate(dID, dCenter, dExtents, Depth));
}

MODE_API void mSpaceDestroy(int ID)
{
	mCHECKVOID(gObjManager.destroy(ID), "Specified SpaceID does not exist.");
}

MODE_API void mHashSpaceSetLevels (int ID, mHashLevels s)
{
	dSpaceID dID = NULL;
	mCHECKVOID(gObjManager.get(ID,dID), "Specified SpaceID does not exist.");
	dHashSpaceSetLevels(dID, s.minLevel, s.maxLevel);
}

MODE_API mHashLevels* mHashSpaceGetLevels (int ID)
{
	dSpaceID dID = NULL;
	mCHECKNULL(gObjManager.get(ID,dID), "Specified SpaceID does not exist.");
		
	int min, max = 0;
	dHashSpaceGetLevels(dID, &min, &max);
	
	mHashLevels* ptr = new mHashLevels;
	ptr->minLevel = min;
	ptr->maxLevel = max;
	return ptr;
}

MODE_API void mDeallocateHashLevels(mHashLevels* ptr)
{
	if (ptr) delete ptr;
	ptr = NULL;
}

MODE_API void mSpaceSetCleanup (int ID, int mode)
{
	dSpaceID dID = NULL;
	mCHECKVOID(gObjManager.get(ID,dID), "Specified SpaceID does not exist.");
	dSpaceSetCleanup(dID, mode);
}

MODE_API int mSpaceGetCleanup (int ID)
{
	dSpaceID dID = NULL;
	mCHECKINT(gObjManager.get(ID,dID), "Specified SpaceID does not exist.");

	return dSpaceGetCleanup(dID);
}

MODE_API void mSpaceAdd (int SID, int GID)
{
	dSpaceID dSID = NULL;
	mCHECKVOID(gObjManager.get(SID,dSID), "Specified SpaceID does not exist.");

	dGeomID dGID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");

	dSpaceAdd(dSID, dGID);
}

MODE_API void mSpaceRemove (int SID, int GID)
{
	dSpaceID dSID = NULL;
	mCHECKVOID(gObjManager.get(SID,dSID), "Specified SpaceID does not exist.");

	dGeomID dGID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");

	dSpaceRemove(dSID, dGID);
}

MODE_API int mSpaceQuery (int SID, int GID)
{
	dSpaceID dSID = NULL;
	mCHECKINT(gObjManager.get(SID,dSID), "Specified SpaceID does not exist.");

	dGeomID dGID = NULL;
	mCHECKINT(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");

	return dSpaceQuery(dSID, dGID);
}

MODE_API int mSpaceGetNumGeoms (int SID)
{
	dSpaceID dSID = NULL;
	mCHECKINT(gObjManager.get(SID,dSID), "Specified SpaceID does not exist.");

	return dSpaceGetNumGeoms(dSID);
}

#ifdef __cplusplus
}
#endif