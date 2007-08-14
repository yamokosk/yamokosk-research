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
#include <sstream>
#include <list>

extern ODEManager gObjManager;

using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

MODE_API void mDisplayDebugInfo(void) 
{
 	vector<int> allSpaceIDs = gObjManager.nAllSpaces();
	vector<int> allGeomIDs = gObjManager.nAllGeoms();
	vector<int> allBodyIDs = gObjManager.nAllBodies();

	// Print a lot of stuff concerning the state of our global variables
	ostringstream outs;

	outs << "DEBUG DATA:" << endl;
	outs << "	Spaces: " << allSpaceIDs.size() << endl;
	outs << "	Geoms: " << allGeomIDs.size() << endl;
	outs << "	Bodies: " << allBodyIDs.size() << endl;
	//outs << endl << "GEOM DUMP:" << endl;

	/*for (int GeomIndex = 0; GeomIndex < gObjManager.count<dGeomID>(); ++GeomIndex)
	{
		outs << "Geom ID: " << allGeomIDs[GeomIndex] << endl;
		outs << "	Belongs in (SpaceID): " << mGeomGetSpace(allGeomIDs[GeomIndex]) << endl;
		outs << "	Geom Class: " << mGeomGetClass(allGeomIDs[GeomIndex]) << endl;
		outs << "	Enabled?: " << mGeomIsEnabled(allGeomIDs[GeomIndex]) << endl;
	}

	outs << endl << "SPACE DUMP:" << endl;

	for (int SpaceIndex = 0; SpaceIndex < gObjManager.count<dSpaceID>(); ++SpaceIndex)
	{
		outs << "Space ID: " << allSpaceIDs[SpaceIndex] << endl;
		outs << "	Num Geoms: " << mSpaceGetNumGeoms(allSpaceIDs[SpaceIndex]) << endl;
		outs << "	Geom IDs: [";
		for (int GeomIndex = 0; GeomIndex < gObjManager.count<dGeomID>(); ++GeomIndex)
		{
			if ( mSpaceQuery(allSpaceIDs[SpaceIndex], allGeomIDs[GeomIndex]) ) outs << allGeomIDs[GeomIndex] << ", ";
		}
		outs << "]" << endl;
	}
	outs << endl;*/
	
	mexPrintf( outs.str().c_str() );
}

#ifdef __cplusplus
}
#endif


#ifdef _DEBUG

#include "drawstuff/drawstuff.h"

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#define Z_OFFSET 0		// z offset for drawing (to get above ground)

#ifdef __cplusplus
extern "C" {
#endif


//****************************************************************************
// graphics
int space_pressed = 0;
int draw_all_objects_called;

void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
	int i,j,n;
	const int N = 100;
	dContactGeom contact[N];

	if (dGeomGetClass (o2) == dRayClass) {
		n = dCollide (o2,o1,N,&contact[0],sizeof(dContactGeom));
	} else {
		n = dCollide (o1,o2,N,&contact[0],sizeof(dContactGeom));
	}
	if (n > 0) {
		dMatrix3 RI;
		dRSetIdentity (RI);
		const dReal ss[3] = {0.01,0.01,0.01};
		for (i=0; i<n; i++) {
			contact[i].pos[2] += Z_OFFSET;
			dsDrawBox (contact[i].pos,RI,ss);
			dVector3 n;
			for (j=0; j<3; j++) n[j] = contact[i].pos[j] + 0.1*contact[i].normal[j];
			dsDrawLine (contact[i].pos,n);
		}
	}
}

void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
	if (!g) return;
	if (dGeomGetClass (g) != dPlaneClass) {
		if (!pos) pos = dGeomGetPosition (g);
		if (!R) R = dGeomGetRotation (g);
	}

	switch (dGeomGetClass (g)) {
		case dSphereClass: {
			dsSetColorAlpha (1,0,0,0.8);
			dReal radius = dGeomSphereGetRadius (g);
			dsDrawSphere (pos, R, radius);
			break;
		}

		case dBoxClass: {
			dsSetColorAlpha (1,1,0,0.8);
			dVector3 sides;
			dGeomBoxGetLengths (g, sides);
			dsDrawBox (pos, R, sides);
			break;
		}

		case dCapsuleClass: {
			dsSetColorAlpha (0,1,0,0.8);
			dReal radius,length;
			dGeomCapsuleGetParams (g, &radius, &length);
			dsDrawCapsule (pos, R, length, radius);
			break;
		}

		/*case dPlaneClass: {
			dVector4 n;
			dMatrix3 R,sides;
			dVector3 pos2,pos3;
			for (int j=0; j<3; ++j) pos3[j] = pos[j];
			dGeomPlaneGetParams (g,n);
			dRFromZAxis (R,n[0],n[1],n[2]);
			for (int j=0; j<3; j++) pos3[j] = n[j]*n[3];
			pos3[2] += Z_OFFSET;
			sides[0] = 2;
			sides[1] = 2;
			sides[2] = 0.001;
			dsSetColor (1,0,1);
			for (int j=0; j<3; j++) pos2[j] = pos3[j] + 0.1*n[j];
			dsDrawLine (pos3,pos2);
			dsSetColorAlpha (1,0,1,0.8);
			dsDrawBox (pos3,R,sides);
			break;
		}*/

		case dGeomTransformClass: {
			// Get transformed geom
			dGeomID g2 = dGeomTransformGetGeom (g);
			const dReal *P_T_g0 = dGeomGetPosition (g2);
			const dReal *R_T_G = dGeomGetRotation (g2);
						
			// Position vector to encapsulated geom in WCS
			dVector3 P_WCS_g0;
			dMULTIPLY0_331 (P_WCS_g0,R,P_T_g0);
			for (int n=0; n<3; ++n) P_WCS_g0[n] += pos[n];
			
			// Rotation matrix of encapsulated geom in WCS
			dMatrix3 R_WCS_G;
			dMULTIPLY0_333 (R_WCS_G, R, R_T_G);
			
			// Recursive call
			drawGeom (g2, P_WCS_g0, R_WCS_G, 0);
		}
	}
	
	if (show_aabb) {
		// draw the bounding box for this geom
		dReal aabb[6];
		dGeomGetAABB (g,aabb);
		dVector3 bbpos;
		for (int i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
		dVector3 bbsides;
		for (int j=0; j<3; j++) bbsides[j] = aabb[j*2+1] - aabb[j*2];
		dMatrix3 RI;
		dRSetIdentity (RI);
		dsSetColorAlpha (1,0,0,0.5);
		dsDrawBox (bbpos,RI,bbsides);
	}
}

void draw_all_objects(dSpaceID space)
{
	int i, j;

	draw_all_objects_called = 1;
	//if (!graphical_test) return;
	int n = dSpaceGetNumGeoms(space);

	// draw all contact points
	//dsSetColor (0,1,1);
	//dSpaceCollide2 (s1,s2,0,&nearCallback);

	// draw all rays
	/*for (i=0; i<n; i++) {
		dGeomID g = dSpaceGetGeom (space,i);
		if (dGeomGetClass (g) == dRayClass) {
			dsSetColor (1,1,1);
			dVector3 origin,dir;
			dGeomRayGet (g,origin,dir);
			origin[2] += Z_OFFSET;
			dReal length = dGeomRayGetLength (g);
			for (j=0; j<3; j++) dir[j] = dir[j]*length + origin[j];
			dsDrawLine (origin,dir);
			dsSetColor (0,0,1);
			dsDrawSphere (origin,dGeomGetRotation(g),0.01);
		}
	}*/

	// draw all other objects
	for (i=0; i<n; i++) {
		dGeomID g = dSpaceGetGeom (space,i);
		
		drawGeom(g,NULL,NULL,0);

		
	}
}

// start simulation - set viewpoint
static void start()
{
	static float xyz[3] = {0,0,23};
	static float hpr[3] = {141.5000,-18.5000,0.0000};
	dsSetViewpoint (xyz,hpr);
}


// called when a key pressed
static void command (int cmd)
{
	if (cmd == ' ') space_pressed = 1;
}

// simulation loop
static void simLoop (int pause)
{
	do {
		draw_all_objects_called = 0;
		vector<dSpaceID> allSpaceIDs = gObjManager.dAllSpaces();

		for (int n=0; n < allSpaceIDs.size(); ++n) {
			if (n == 0) dsSetColor (0,1,1);
			if (n == 1) dsSetColor (1,0,1);
			if (n == 2) dsSetColor (1,1,0);
			draw_all_objects(allSpaceIDs[n]);
		}
	} while (!draw_all_objects_called);
}

MODE_API void mDisplayDebugWindow(void) 
{
	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = "D:/usr/bin/ode-0.8/drawstuff/textures";

	dsSetSphereQuality (3);
	dsSetCapsuleQuality (8);
	int argc = 0;
	char** argv = NULL;
	dsSimulationLoop (argc,argv,352,288,&fn);
}



#ifdef __cplusplus
}
#endif

#else

#ifdef __cplusplus
extern "C" {
#endif


MODE_API void mDisplayDebugWindow(void) {}

#ifdef __cplusplus
}
#endif


#endif