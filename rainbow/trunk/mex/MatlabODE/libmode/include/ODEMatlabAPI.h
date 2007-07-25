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

#ifndef ODEMATLABAPI_HEADER_FILE
#define ODEMATLABAPI_HEADER_FILE

#include "mex.h"

#if defined MEXCDETECT_EXPORTS
	#define MODE_API __declspec(dllexport)
#else
	#define MODE_API
#endif

#ifdef __cplusplus
extern "C" {
#endif
	
struct mContactGeom {
	mxArray* pos;
	mxArray* normal;
	mxArray* depth;
	int geom1, geom2;
	int side1, side2;
};

struct mHashLevels {
	int minLevel;
	int maxLevel;
};

struct mContactSpace {
	mContactGeom* pContacts;
};

struct mTriMeshData {
	mxArray* vertices;
	mxArray* indices;
	mxArray* normals;
};

// Rigid body function wrappers
MODE_API int mBodyCreate (void);
MODE_API void mBodyDestroy (int);
MODE_API void mBodySetPosition   (int, double, double, double);
MODE_API void mBodySetRotation   (int, const mxArray*);
MODE_API void mBodySetQuaternion (int, const mxArray*);
MODE_API mxArray* mBodyGetPosition   (int);
MODE_API mxArray* mBodyGetRotation   (int);
MODE_API mxArray* mBodyGetQuaternion (int);
MODE_API mxArray* mBodyGetRelPointPos (int, double, double, double);
MODE_API mxArray* mBodyGetPosRelPoint (int, double, double, double);
MODE_API mxArray* mBodyVectorToWorld   (int, double, double, double);
MODE_API mxArray* mBodyVectorFromWorld (int, double, double, double);

// Geom function wrappers
MODE_API void mGeomDestroy (int);
//MODE_API void mGeomSetData (int, void *);
//MODE_API void mGeomGetData (int, void *);
MODE_API void mGeomSetBody (int, int);
MODE_API int mGeomGetBody (int);
MODE_API void mGeomSetPosition (int, double, double, double);
MODE_API void mGeomSetRotation (int, const mxArray*);
MODE_API void mGeomSetQuaternion (int, const mxArray*);
MODE_API mxArray* mGeomGetPosition (int);
MODE_API mxArray* mGeomGetRotation (int);
MODE_API mxArray* mGeomGetQuaternion (int);
MODE_API mxArray* mGeomGetAABB (int);
MODE_API int mGeomIsSpace (int);
MODE_API int mGeomGetSpace (int);
MODE_API int mGeomGetClass (int);
MODE_API void mGeomSetCategoryBits (int, unsigned long);
MODE_API void mGeomSetCollideBits (int, unsigned long);
MODE_API unsigned long mGeomGetCategoryBits (int);
MODE_API unsigned long mGeomGetCollideBits (int);
MODE_API void mGeomEnable (int);
MODE_API void mGeomDisable (int);
MODE_API int mGeomIsEnabled (int);

// Collision function wrappers
MODE_API bool mSimpleCollide(int, int);
MODE_API mContactGeom* mCollide(int, int, int);
MODE_API bool mSimpleSpaceCollide(int);
MODE_API mxArray* mSpaceCollide(int);
MODE_API bool mSimpleSpaceCollide2(int, int);
MODE_API mxArray* mSpaceCollide2(int, int);
MODE_API void mDeallocateGeomContact(mContactGeom*);

// Space function wrappers
MODE_API int mSimpleSpaceCreate(int);
MODE_API int mHashSpaceCreate (int);
MODE_API int mQuadTreeSpaceCreate (int, const mxArray*, const mxArray*, int Depth);
MODE_API void mSpaceDestroy (int);
MODE_API void mHashSpaceSetLevels (int, mHashLevels);
MODE_API mHashLevels* mHashSpaceGetLevels (int);
MODE_API void mDeallocateHashLevels(mHashLevels*);
MODE_API void mSpaceSetCleanup (int, int);
MODE_API int mSpaceGetCleanup (int);
MODE_API void mSpaceAdd (int, int);
MODE_API void mSpaceRemove (int, int);
MODE_API int mSpaceQuery (int, int);
MODE_API int mSpaceGetNumGeoms (int);

// Geometry class wrappers - Sphere
MODE_API int mCreateSphere(int, double);
MODE_API void mGeomSphereSetRadius(int, double);
MODE_API double mGeomSphereGetRadius(int);
MODE_API double mGeomSpherePointDepth(int, double, double, double);

// Geometry class wrappers - Capped cylinder
MODE_API int mCreateCCylinder (int, double, double);
MODE_API void mGeomCCylinderSetParams (int, double, double);
MODE_API mxArray* mGeomCCylinderGetParams (int);
MODE_API double mGeomCCylinderPointDepth (int, double, double, double);

// Geometry class wrappers - Plane
MODE_API int mCreatePlane (int, double, double, double, double);
MODE_API void mGeomPlaneSetParams (int, double, double, double, double);
MODE_API mxArray* mGeomPlaneGetParams (int);
MODE_API double mGeomPlanePointDepth (int, double, double, double);

// Geometry class wrappers - Box
MODE_API int mCreateBox (int, double, double, double);
MODE_API void mGeomBoxSetLengths (int, double, double, double);
MODE_API mxArray* mGeomBoxGetLengths (int);
MODE_API double mGeomBoxPointDepth (int, double, double, double);

// Geometry class wrappers - Triangle Mesh
MODE_API int mCreateTriMesh (int, const mTriMeshData*);

// Debug/Testing functions
MODE_API void mDisplayDebugInfo(void);

#ifdef __cplusplus
}
#endif

#endif
