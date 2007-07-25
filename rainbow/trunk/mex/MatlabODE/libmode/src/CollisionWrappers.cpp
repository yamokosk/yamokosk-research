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
#include <utility>
#include <vector>

extern ODEManager gObjManager;

using namespace std;

typedef vector< pair<int,int> > PairContainer;

#ifdef __cplusplus
extern "C" {
#endif

MODE_API bool mSimpleCollide(int ID1, int ID2)
{
	// Locate the ODE geom objects from their IDS
	dGeomID o1 = NULL, o2 = NULL;
	mxAssert(gObjManager.get(ID1,o1), "Specified GeomID does not exist.");
	mxAssert(gObjManager.get(ID2,o2), "Specified GeomID does not exist.");

	// Call ODE dCollide
	dContactGeom dContactPts;
	int numContactPts = dCollide(o1, o2, 1, &dContactPts, sizeof(dContactGeom));
		
	if (numContactPts > 0) return true;
	else return false;
}

MODE_API mContactGeom* mCollide(int ID1, int ID2, int flags)
{
	// Locate the ODE geom objects from their IDS
	dGeomID o1 = NULL, o2 = NULL;
	mxAssert(gObjManager.get(ID1,o1), "Specified GeomID does not exist.");
	mxAssert(gObjManager.get(ID2,o2), "Specified GeomID does not exist.");

	// Grab lower 16 bits of flags
	int numDesiredContactPts = (flags & 0x0000FFFF);

	// Call ODE dCollide
	dContactGeom* dContactPts = new dContactGeom[numDesiredContactPts];
	int numContactPts = dCollide(o1, o2, flags, dContactPts, sizeof(dContactGeom));
	if (numContactPts == 0) return NULL;

	// Create and populate output mxArray arrays
	mxArray* pos = mxCreateDoubleMatrix(3,numContactPts, mxREAL); double* posPtr = mxGetPr(pos);
	mxArray* normal = mxCreateDoubleMatrix(3,numContactPts, mxREAL); double* normalPtr = mxGetPr(normal);
	mxArray* depth = mxCreateDoubleMatrix(1,numContactPts, mxREAL); double* depthPtr = mxGetPr(depth);
	
	for (int n=0; n < numContactPts; ++n) {
		for (int m=0; m < 3; ++m) {
			posPtr[n*3 + m] = dContactPts[n].pos[m];
			normalPtr[n*3 + m] = dContactPts[n].normal[m];
		}
		depthPtr[n] = dContactPts[n].depth;
	}

	// Clean up
	delete [] dContactPts;

	mContactGeom* contactData = new mContactGeom;
	contactData->pos = pos;
	contactData->normal = normal;
	contactData->depth = depth;
	contactData->geom1 = ID1;
	contactData->geom2 = ID2;
	contactData->side1 = 0;
	contactData->side2 = 0;

	return contactData;
}

MODE_API void mDeallocateGeomContact(mContactGeom* ptr)
{
	if (ptr) {
		mxDestroyArray(ptr->pos);
		mxDestroyArray(ptr->normal);
		mxDestroyArray(ptr->depth);
		delete ptr;
	}
	ptr = NULL;
}

void SimpleCallback(void* data, dGeomID o1, dGeomID o2)
{
	// TODO: If the Geom is a TriMesh, I might need to be setting the previous
	// transform with calls to void dGeomTriMeshSetLastTransform( dGeomID g, dMatrix4 last_trans )
	// and dReal* dGeomTriMeshGetLastTransform( dGeomID g )
	bool* bVal = (bool*)data;

	if (!(*bVal)) {
		dContactGeom dContactPts;
		int numContactPts = dCollide(o1, o2, 1, &dContactPts, sizeof(dContactGeom));

		if (numContactPts > 0) *bVal = true;
	} else return;
}

void DetailedCallback(void* data, dGeomID o1, dGeomID o2)
{
	// TODO: If the Geom is a TriMesh, I might need to be setting the previous
	// transform with calls to void dGeomTriMeshSetLastTransform( dGeomID g, dMatrix4 last_trans )
	// and dReal* dGeomTriMeshGetLastTransform( dGeomID g )
	PairContainer* pCollisionPairs = (PairContainer*)data;

	dContactGeom dContactPts;
	int numContactPts = dCollide(o1, o2, 1, &dContactPts, sizeof(dContactGeom));

	if (numContactPts > 0) {
		int nO1, nO2;
		gObjManager.id(nO1, o1);
		gObjManager.id(nO2, o2);
		pCollisionPairs->push_back( pair<int,int>(nO1, nO2) );
	}
}

MODE_API bool mSimpleSpaceCollide(int SID)
{
	dSpaceID dSID = NULL;
	mxAssert(gObjManager.get(SID,dSID), "Specified SpaceID does not exist.");

	bool bSelfCollision = false;
	dSpaceCollide (dSID, (void*)&bSelfCollision, SimpleCallback);
	return bSelfCollision;
}

MODE_API mxArray* mSpaceCollide(int SID)
{
	dSpaceID dSID = NULL;
	mxAssert(gObjManager.get(SID,dSID), "Specified SpaceID does not exist.");

	PairContainer CollisionPairs;
	dSpaceCollide (dSID, (void*)&CollisionPairs, DetailedCallback);

	mxArray* pairs = mxCreateDoubleMatrix(CollisionPairs.size(),2, mxREAL); double* pPairs = mxGetPr(pairs);

	for (unsigned int m=0; m < CollisionPairs.size(); ++m) {
		pPairs[m*2] = CollisionPairs[m].first;
		pPairs[m*2 + 1] = CollisionPairs[m].second;
	}

	return pairs;
}

MODE_API bool mSimpleSpaceCollide2(int SID1, int SID2)
{
	dSpaceID dSID1 = NULL, dSID2 = NULL;
	mxAssert( gObjManager.get(SID1,dSID1) && gObjManager.get(SID2,dSID2), "Specified SpaceID does not exist.");

	bool bSelfCollision = false;
	dSpaceCollide2((dGeomID)dSID1, (dGeomID)dSID2, (void*)&bSelfCollision, SimpleCallback);
	return bSelfCollision;
}

MODE_API mxArray* mSpaceCollide2(int SID1, int SID2)
{
	dSpaceID dSID1 = NULL, dSID2 = NULL;
	mxAssert( gObjManager.get(SID1,dSID1) && gObjManager.get(SID2,dSID2), "Specified SpaceID does not exist.");

	PairContainer CollisionPairs;
	dSpaceCollide2((dGeomID)dSID1, (dGeomID)dSID2, (void*)&CollisionPairs, DetailedCallback);

	mxArray* pairs = mxCreateDoubleMatrix(2,CollisionPairs.size(), mxREAL); double* pPairs = mxGetPr(pairs);

	for (unsigned int m=0; m < CollisionPairs.size(); ++m) {
		pPairs[m*2] = CollisionPairs[m].first;
		pPairs[m*2 + 1] = CollisionPairs[m].second;
	}

	return pairs;
}

#ifdef __cplusplus
}
#endif