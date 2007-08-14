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
	mCHECKVOID(gObjManager.destroy(ID), "Could not destroy the specified Body. Does it exist?");
}

MODE_API void mBodySetPosition   (int BID, double x, double y, double z)
{
	dBodyID dBID = NULL;
	mCHECKVOID(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dBodySetPosition(dBID, x, y, z);
}

MODE_API void mBodySetRotation   (int BID, const mxArray* R)
{
	dBodyID dBID = NULL;
	mCHECKVOID(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dMatrix3 dR = {0};
	double *ptr = mxGetPr(R);
	//MEMCPY(dR, mxGetPr(R), 3*3);
	// Note 1. R from Matlab should already be transposed.. no need to do it again
	// Note 2. dR (for some stupid reason) is a 3*4 container...
	int ind = 0;
	for (int n=0; n < 11; ++n) {
		if ( (n != 3) && (n != 7) ) {
			dR[n] = ptr[ind];
			ind++;
		}
	}
	//dR[0] = ptr[0];	 // dR[0][0] = ptr[0][0]
	//dR[1] = ptr[1];	 // dR[0][1] = ptr[1][0]
	//dR[2] = ptr[2];	 // dR[0][2] = ptr[2][0]
	// dR[3]		 // dR[0][3] = NA
	//dR[4] = ptr[3];  // dR[1][0] = ptr[0][1]
	//dR[5] = ptr[4];  // dR[1][1] = ptr[1][1]
	//dR[6] = ptr[5];  // dR[1][2] = ptr[2][1]
	// dR[7]		 // dR[1][3] = NA
	//dR[8] = ptr[6];  // dR[2][0] = ptr[0][2]
	//dR[9] = ptr[7];  // dR[2][1] = ptr[1][2]
	//dR[10] = ptr[8]; // dR[2][2] = ptr[2][2]
	// dR[11]		 // dR[2][3] = NA

	//mexPrintf("\n(%f, %f, %f, %f)\n", dR[0], dR[1], dR[2], dR[3]);
	//mexPrintf("(%f, %f, %f, %f)\n", dR[4], dR[5], dR[6], dR[7]);
	//mexPrintf("(%f, %f, %f, %f)\n", dR[8], dR[9], dR[10], dR[11]);

	dBodySetRotation(dBID, dR);
}

MODE_API void mBodySetQuaternion (int BID, const mxArray* q)
{
	dBodyID dBID = NULL;
	mCHECKVOID(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dQuaternion dq = {0};
	MEMCPY(dq, mxGetPr(q), 4);

	dBodySetQuaternion(dBID, dq);
}

MODE_API mxArray* mBodyGetPosition   (int BID)
{
	dBodyID dBID = NULL;
	mCHECKNULL(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	const dReal* dPos = dBodyGetPosition(dBID);
	mCHECKNULL(dPos, "Bad return from ODE");

	mxArray* mPos = mxCreateDoubleMatrix(3,1,mxREAL);
	double* pmPos = mxGetPr(mPos);
	
	MEMCPY(pmPos, dPos, 3);
	
	return mPos;
}

MODE_API mxArray* mBodyGetRotation   (int BID)
{
	dBodyID dBID = NULL;
	mCHECKNULL(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	mxArray* mR = mxCreateDoubleMatrix(3,3,mxREAL);
	double* ptr = mxGetPr(mR);

	const dReal* dR = dBodyGetRotation(dBID);
	//MEMCPY(pmR, dR, 3*3);
	
	//mexPrintf("\n(%f, %f, %f, %f)\n", dR[0], dR[1], dR[2], dR[3]);
	//mexPrintf("(%f, %f, %f, %f)\n", dR[4], dR[5], dR[6], dR[7]);
	//mexPrintf("(%f, %f, %f, %f)\n", dR[8], dR[9], dR[10], dR[11]);

	int ind = 0;
	for (int n=0; n < 11; ++n) {
		if ( (n != 3) && (n != 7) ) {
			ptr[ind] = dR[n];
			ind++;
		}
	}

	return mR;
}

MODE_API mxArray* mBodyGetQuaternion (int BID)
{
	dBodyID dBID = NULL;
	mCHECKNULL(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	mxArray* mq = mxCreateDoubleMatrix(4,1,mxREAL);
	double* pmq = mxGetPr(mq);

	const dReal* dq = dBodyGetRotation(dBID);
	MEMCPY(pmq, dq, 4);
	
	return mq;
}

MODE_API mxArray* mBodyGetRelPointPos (int BID, double x, double y, double z)
{
	dBodyID dBID = NULL;
	mCHECKNULL(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dVector3 dv = {0};
	dBodyGetRelPointPos(dBID, x, y, z, dv);

	mxArray* v = mxCreateDoubleMatrix(3,1,mxREAL);
	MEMCPY(mxGetPr(v), dv, 3);

	return v;
}

MODE_API mxArray* mBodyGetPosRelPoint (int BID, double x, double y, double z)
{
	dBodyID dBID = NULL;
	mCHECKNULL(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dVector3 dv = {0};
	dBodyGetPosRelPoint(dBID, x, y, z, dv);

	mxArray* v = mxCreateDoubleMatrix(3,1,mxREAL);
	MEMCPY(mxGetPr(v), dv, 3);

	return v;
}

MODE_API mxArray* mBodyVectorToWorld   (int BID, double x, double y, double z)
{
	dBodyID dBID = NULL;
	mCHECKNULL(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dVector3 dv = {0};
	dBodyVectorToWorld(dBID, x, y, z, dv);

	mxArray* v = mxCreateDoubleMatrix(3,1,mxREAL);
	MEMCPY(mxGetPr(v), dv, 3);

	return v;
}

MODE_API mxArray* mBodyVectorFromWorld (int BID, double x, double y, double z)
{
	dBodyID dBID = NULL;
	mCHECKNULL(gObjManager.get(BID,dBID), "Specified BodyID does not exist.");

	dVector3 dv = {0};
	dBodyVectorFromWorld(dBID, x, y, z, dv);

	mxArray* v = mxCreateDoubleMatrix(3,1,mxREAL);
	MEMCPY(mxGetPr(v), dv, 3);

	return v;
}

#ifdef __cplusplus
}
#endif
