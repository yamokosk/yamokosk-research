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
//	 Lesser General Public License for more details.
// 
//   You should have received a copy of the GNU Lesser General Public
//   License along with this library; if not, write to the Free Software
//   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
//   ODE is Copyright © 2001-2004 Russell L. Smith. All rights reserved.

#include "ODEMatlabAPI.h"
#include "ODEManager.h"

extern ODEManager gObjManager;

#ifdef __cplusplus
extern "C" {
#endif

MODE_API int mBodyCreate (void)
{
	return gObjManager.add(dBodyCreate(gObjManager.mWorld));
}

MODE_API void mBodyDestroy (int ID)
{
	mxAssert(gObjManager.destroy(ID), "Could not destroy the specified Body. Does it exist?");
}

MODE_API void mBodySetPosition   (int BID, double x, double y, double z)
{
	dBodyID dBID = NULL;
	mxAssert(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dBodySetPosition(dBID, x, y, z);
}

MODE_API void mBodySetRotation   (int BID, const mxArray* R)
{
	dBodyID dBID = NULL;
	mxAssert(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dMatrix3 dR = {0};
	MEMCPY(dR, mxGetPr(R), 3*3);
	
	dBodySetRotation(dBID, dR);
}

MODE_API void mBodySetQuaternion (int BID, const mxArray* q)
{
	dBodyID dBID = NULL;
	mxAssert(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dQuaternion dq = {0};
	MEMCPY(dq, mxGetPr(q), 4);

	dBodySetQuaternion(dBID, dq);
}

MODE_API mxArray* mBodyGetPosition   (int BID)
{
	dBodyID dBID = NULL;
	mxAssert(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	const dReal* dPos = dBodyGetPosition(dBID);
	mxAssert(dPos, "Bad return from ODE");

	mxArray* mPos = mxCreateDoubleMatrix(3,1,mxREAL);
	double* pmPos = mxGetPr(mPos);
	
	MEMCPY(pmPos, dPos, 3);
	
	return mPos;
}

MODE_API mxArray* mBodyGetRotation   (int BID)
{
	dBodyID dBID = NULL;
	mxAssert(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	mxArray* mR = mxCreateDoubleMatrix(3,3,mxREAL);
	double* pmR = mxGetPr(mR);

	const dReal* dR = dBodyGetRotation(dBID);
	MEMCPY(pmR, dR, 3*3);
	
	return mR;
}

MODE_API mxArray* mBodyGetQuaternion (int BID)
{
	dBodyID dBID = NULL;
	mxAssert(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	mxArray* mq = mxCreateDoubleMatrix(4,1,mxREAL);
	double* pmq = mxGetPr(mq);

	const dReal* dq = dBodyGetRotation(dBID);
	MEMCPY(pmq, dq, 4);
	
	return mq;
}

MODE_API mxArray* mBodyGetRelPointPos (int BID, double x, double y, double z)
{
	dBodyID dBID = NULL;
	mxAssert(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dVector3 dv = {0};
	dBodyGetRelPointPos(dBID, x, y, z, dv);

	mxArray* v = mxCreateDoubleMatrix(3,1,mxREAL);
	MEMCPY(mxGetPr(v), dv, 3);

	return v;
}

MODE_API mxArray* mBodyGetPosRelPoint (int BID, double x, double y, double z)
{
	dBodyID dBID = NULL;
	mxAssert(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dVector3 dv = {0};
	dBodyGetPosRelPoint(dBID, x, y, z, dv);

	mxArray* v = mxCreateDoubleMatrix(3,1,mxREAL);
	MEMCPY(mxGetPr(v), dv, 3);

	return v;
}

MODE_API mxArray* mBodyVectorToWorld   (int BID, double x, double y, double z)
{
	dBodyID dBID = NULL;
	mxAssert(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dVector3 dv = {0};
	dBodyVectorToWorld(dBID, x, y, z, dv);

	mxArray* v = mxCreateDoubleMatrix(3,1,mxREAL);
	MEMCPY(mxGetPr(v), dv, 3);

	return v;
}

MODE_API mxArray* mBodyVectorFromWorld (int BID, double x, double y, double z)
{
	dBodyID dBID = NULL;
	mxAssert(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dVector3 dv = {0};
	dBodyVectorFromWorld(dBID, x, y, z, dv);

	mxArray* v = mxCreateDoubleMatrix(3,1,mxREAL);
	MEMCPY(mxGetPr(v), dv, 3);

	return v;
}

#ifdef __cplusplus
}
#endif
