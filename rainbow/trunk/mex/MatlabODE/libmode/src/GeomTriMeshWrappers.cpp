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

#ifdef __cplusplus
extern "C" {
#endif

MODE_API int mCreateTriMesh (int SID, const mTriMeshData* pMeshData)
{
	dSpaceID dSID = NULL;
	if (SID != 0) mCHECKINT(gObjManager.get(SID,dSID), "Specified GeomID does not exist.");

	// Create a copy of the incoming data.. ODE is too lazy to do this itself
	int VertexCount = mxGetN( pMeshData->vertices );
	int IndexCount = mxGetM( pMeshData->indices );

	TriMeshDataStorage LocalDataStorage;
	LocalDataStorage.vertices = new dVector3[VertexCount];
	LocalDataStorage.indices = new int[IndexCount];

	MEMCPY(LocalDataStorage.indices, mxGetPr( pMeshData->indices ), IndexCount );
	double* pmVertexData = mxGetPr( pMeshData->vertices );
	for (int c=0; c < VertexCount; ++c) {
		for (int n=0; n < 3; ++n)
			LocalDataStorage.vertices[c][n] = pmVertexData[c*3 + n];
	}
	gObjManager.add( LocalDataStorage ); // Sucks, but I have to keep this data persistent

	dTriMeshDataID dMeshData = dGeomTriMeshDataCreate();
	dGeomTriMeshDataBuildSimple( dMeshData, 
								 (dReal*)LocalDataStorage.vertices, VertexCount, 
								 LocalDataStorage.indices, IndexCount);
	gObjManager.add( dMeshData ); // Sucks, but I have to keep this data persistent

	dGeomID dGID = dCreateTriMesh(dSID, dMeshData, NULL, NULL, NULL);
	dGeomTriMeshEnableTC(dGID, dGeomGetClass(dGID), 0); // Make sure temporal coherence is turned OFF

	return gObjManager.add( dGID );
}

#ifdef __cplusplus
}
#endif