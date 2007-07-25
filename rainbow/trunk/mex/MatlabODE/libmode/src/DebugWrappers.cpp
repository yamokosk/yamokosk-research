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

#ifdef _DEBUG

#ifdef __cplusplus
extern "C" {
#endif

MODE_API void mDisplayDebugInfo(void) 
{
 	vector<int> allSpaceIDs = gObjManager.all<dSpaceID>();
	vector<int> allGeomIDs = gObjManager.all<dGeomID>();
	vector<int> allBodyIDs = gObjManager.all<dBodyID>();

	// Print a lot of stuff concerning the state of our global variables
	ostringstream outs;

	outs << "DEBUG DATA:" << endl;
	outs << "	Spaces: " << gObjManager.count<dSpaceID>() << endl;
	outs << "	Geoms: " << gObjManager.count<dGeomID>() << endl;
	outs << "	Bodies: " << gObjManager.count<dBodyID>() << endl;
	outs << endl << "GEOM DUMP:" << endl;

	for (int GeomIndex = 0; GeomIndex < gObjManager.count<dGeomID>(); ++GeomIndex)
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
	outs << endl;
	
	mexPrintf( outs.str().c_str() );
}


#ifdef __cplusplus
}
#endif

#else

#ifdef __cplusplus
extern "C" {
#endif


MODE_API void mDisplayDebugInfo(void) {}


#ifdef __cplusplus
}
#endif


#endif