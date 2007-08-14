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

ODEManager gObjManager;

#ifdef __cplusplus
extern "C" {
#endif

MODE_API void mGeomDestroy (int id) 
{
	gObjManager.destroy(id);	
}

MODE_API void mGeomSetBody (int GID, int BID)
{
	dGeomID dGID = NULL; dBodyID dBID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	mCHECKVOID(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");
	dGeomSetBody(dGID, dBID);
}

MODE_API int mGeomGetBody (int GID)
{
	dGeomID dGID = NULL;
	mCHECKINT(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	dBodyID dBID = dGeomGetBody(dGID);
	
	int BID = 0;
	mCHECKINT(gObjManager.id(BID,dBID), "Returned BodyID does not exist. (Should never happen!)");
	// TODO: If this assert ever returns an error, the Manager class has lost track 
	// of an object. Probably need to implement a cleanup/verification between the
	// manager and ODE
	return BID;
}

MODE_API void mGeomSetPosition (int GID, double x, double y, double z)
{
	dGeomID dGID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	dGeomSetPosition(dGID, x, y, z);
}

MODE_API void mGeomSetRotation (int GID, const mxArray* R)
{
	dGeomID dGID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");

	dMatrix3 dR = {0};
	double *ptr = mxGetPr(R);
	//MEMCPY(dR, mxGetPr(R), 3*3);
	int ind = 0;
	for (int n=0; n < 11; ++n) {
		if ( (n != 3) && (n != 7) ) {
			dR[n] = ptr[ind];
			ind++;
		}
	}
	
	dGeomSetRotation(dGID, dR);
}

MODE_API void mGeomSetQuaternion (int GID, const mxArray* q)
{
	dGeomID dGID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");

	dQuaternion dq = {0};
	MEMCPY(dq, mxGetPr(q), 4);

	dGeomSetQuaternion(dGID, dq);
}

MODE_API mxArray* mGeomGetPosition (int GID)
{
	dGeomID dGID = NULL;
	mCHECKNULL(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	
	mxArray* mPos = mxCreateDoubleMatrix(3,1,mxREAL);
	double* pmPos = mxGetPr(mPos);

	const dReal* dPos = dGeomGetPosition(dGID);
	//for(int n=0; n<3; ++n) pmPos[n] = dPos[n];
	MEMCPY(pmPos, dPos, 3);
	
	return mPos;
}

MODE_API mxArray* mGeomGetRotation (int GID)
{
	dGeomID dGID = NULL;
	mCHECKNULL(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	
	mxArray* mR = mxCreateDoubleMatrix(3,3,mxREAL);
	double* ptr = mxGetPr(mR);

	const dReal* dR = dGeomGetRotation(dGID);
	//MEMCPY(pmR, dR, 3*3);
	int ind = 0;
	for (int n=0; n < 11; ++n) {
		if ( (n != 3) && (n != 7) ) {
			ptr[ind] = dR[n];
			ind++;
		}
	}
	
	return mR;
}

MODE_API mxArray* mGeomGetQuaternion (int GID)
{
	dGeomID dGID = NULL;
	mCHECKNULL(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");

	mxArray* mq = mxCreateDoubleMatrix(4,1,mxREAL);
	double* pmq = mxGetPr(mq);

	const dReal* dq = dGeomGetRotation(dGID);
	MEMCPY(pmq, dq, 4);
	
	return mq;
}

MODE_API mxArray* mGeomGetAABB (int GID)
{
	dGeomID dGID = NULL;
	mCHECKNULL(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");

	mxArray* mAABB = mxCreateDoubleMatrix(6,1,mxREAL);
	double* pmAABB = mxGetPr(mAABB);

	dReal dAABB[6] = {0};
	dGeomGetAABB(dGID, dAABB);
	MEMCPY(pmAABB, dAABB, 6);
	
	return mAABB;
}

MODE_API int mGeomIsSpace (int GID)
{
	dGeomID dGID = NULL;
	mCHECKINT(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	return dGeomIsSpace(dGID);
}

MODE_API int mGeomGetSpace (int GID)
{
	dGeomID dGID = NULL;
	mCHECKINT(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");

	dSpaceID dSID = dGeomGetSpace(dGID);
	int SID = 0;
	mCHECKINT(gObjManager.id(SID,dSID), "Specified GeomID does not exist.");
	return SID;
}

MODE_API int mGeomGetClass (int GID)
{
	dGeomID dGID = NULL;
	mCHECKINT(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	return dGeomGetClass(dGID);
}

MODE_API void mGeomSetCategoryBits (int GID, unsigned long bits)
{
	dGeomID dGID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	dGeomSetCategoryBits(dGID, bits);
}

MODE_API void mGeomSetCollideBits (int GID, unsigned long bits)
{
	dGeomID dGID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	dGeomSetCollideBits(dGID, bits);
}

MODE_API unsigned long mGeomGetCategoryBits (int GID)
{
	dGeomID dGID = NULL;
	mCHECKINT(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	return dGeomGetCategoryBits(dGID);
}

MODE_API unsigned long mGeomGetCollideBits (int GID)
{
	dGeomID dGID = NULL;
	mCHECKINT(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	return dGeomGetCollideBits(dGID);
}

MODE_API void mGeomEnable (int GID)
{
	dGeomID dGID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	dGeomEnable(dGID);
}

MODE_API void mGeomDisable (int GID)
{
	dGeomID dGID = NULL;
	mCHECKVOID(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	dGeomDisable(dGID);
}

MODE_API int mGeomIsEnabled (int GID)
{
	dGeomID dGID = NULL;
	mCHECKINT(gObjManager.get(GID,dGID), "Specified GeomID does not exist.");
	return dGeomIsEnabled(dGID);
}

#ifdef __cplusplus
}
#endif