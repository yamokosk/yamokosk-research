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

#ifndef ODEMATLABWRAPPER_HEADER_FILE
#define ODEMATLABWRAPPER_HEADER_FILE

#pragma warning( disable: 4244 ) // Conversions from 'double' to 'dReal', possible loss of data
#pragma warning( disable: 4800 ) // 'const dObjectID *' : forcing value to bool 'true' or 'false' (performance warning)

#include "mex.h"
#include "ode/ode.h"
#include <map>
#include <boost/function.hpp>
#include <vector>
#include <boost/any.hpp>
#include "error.h"

#define MEMCPY( dest, src, n ) \
   for( int i = 0; i <= (n); ++i ) (dest[i]) = (src[i]);

using namespace std;


struct TriMeshDataStorage {
	TriMeshDataStorage() : vertices(NULL), indices(NULL) {};
	dVector3* vertices;
	int* indices;
};

class ODEManager
{
public:
	typedef map<int, boost::any> MapIDObj;
	typedef MapIDObj::iterator MapIDObjIterator;

	ODEManager() : mNumObjs(0) 
	{
		dInitODE(); 
		mWorld = dWorldCreate();
#ifdef _DEBUG
		mexPrintf("MODE Library (Debug) loaded.\n");
#else
		mexPrintf("MODE Library (Release) loaded.\n");
#endif
	}
	~ODEManager() 
	{
		mexPrintf("MODE Library unloading.\n");
		while( !mIDObjMap.empty() ) {
			MapIDObjIterator it = mIDObjMap.begin();
			destroy( it->first );
		}
		dWorldDestroy(mWorld);
		dCloseODE();
	}

	template<class dObject>
	int add(dObject obj) 
	{
		mIDObjMap[++mNumObjs] = obj;
		return mNumObjs;
	};
	
	template<class dObject>
	bool get(int n, dObject& obj)
	{
		MapIDObjIterator it = mIDObjMap.find(n);
		if ( it != mIDObjMap.end() ) {
			obj = boost::any_cast<dObject>(it->second);
			return true;
		}
		return false;			
	};
	
	template<class dObject>
	bool id(int& n, dObject& obj) 
	{
		for (MapIDObjIterator it = mIDObjMap.begin(); it != mIDObjMap.end(); ++it) {
			if (obj == boost::any_cast<dObject>(it->second)) {
				n = it->first;
				return true;
			}
		}
		n = -1;	return false;
	};

	bool destroy(int n) 
	{
		MapIDObjIterator it = mIDObjMap.find(n);
		if ( it != mIDObjMap.end() ) {
			if ( is_geom(it->second) ) dGeomDestroy(boost::any_cast<dGeomID>(it->second));
			if ( is_body(it->second) ) dBodyDestroy(boost::any_cast<dBodyID>(it->second));
			if ( is_space(it->second) ) {
				// Spaces are special.. need to recursively destroy everything contained in it
				dSpaceID s = boost::any_cast<dSpaceID>(it->second);
				vector<int> ag = this->nAllGeoms();
				for (unsigned int ii=0; ii < ag.size(); ++ii) {
					dGeomID g = NULL; this->get(ag[ii],g);
					if ( dSpaceQuery(s, g) ) this->destroy( ag[ii] );
				}
				dSpaceDestroy(s);
			}
			if ( is_dtridata(it->second) ) {
				// I assume we are also going to delete the associated geom?
				dGeomTriMeshDataDestroy( boost::any_cast<dTriMeshDataID>(it->second) );
			}
			if ( is_ltridata(it->second) ) {
				TriMeshDataStorage obj = boost::any_cast<TriMeshDataStorage>(it->second);
				if (obj.vertices) delete [] obj.vertices;
				if (obj.indices) delete [] obj.indices;
			}
			mIDObjMap.erase(n);
			return true;
		}
		return false;
	};
	
	template<class dObject>
	int count() 
	{
		int num = 0;
		for (MapIDObjIterator it = mIDObjMap.begin(); it !=mIDObjMap.end(); ++it) {
			if ( boost::any_cast<dObject>(it->second) ) num++;
		}
		return num;
	};

	//template<class dObject>
	//vector<int> all()
	//{
	//	vector<int> out;
	//	for (MapIDObjIterator it = mIDObjMap.begin(); it !=mIDObjMap.end(); ++it) {
	//		if ( boost::any_cast<dObject>(it->second) ) out.push_back(it->first);
	//	}
	//	return out;
	//};

	vector<dSpaceID> dAllSpaces()
	{
		vector<dSpaceID> out;
		for (MapIDObjIterator it = mIDObjMap.begin(); it !=mIDObjMap.end(); ++it) {
			if ( is_space(it->second) ) out.push_back( boost::any_cast<dSpaceID>(it->second) );
		}
		return out;
	};

	vector<int> nAllSpaces()
	{
		vector<int> out;
		for (MapIDObjIterator it = mIDObjMap.begin(); it !=mIDObjMap.end(); ++it) {
			if ( is_space(it->second) ) out.push_back(it->first);
		}
		return out;
	};

	vector<int> nAllGeoms()
	{
		vector<int> out;
		for (MapIDObjIterator it = mIDObjMap.begin(); it !=mIDObjMap.end(); ++it) {
			if ( is_geom(it->second) ) out.push_back(it->first);
		}
		return out;
	};

	vector<int> nAllBodies()
	{
		vector<int> out;
		for (MapIDObjIterator it = mIDObjMap.begin(); it !=mIDObjMap.end(); ++it) {
			if ( is_body(it->second) ) out.push_back(it->first);
		}
		return out;
	};

	dWorldID mWorld;

protected:
	bool is_geom(const boost::any& operand) {return boost::any_cast<dGeomID>(&operand);};
	bool is_space(const boost::any& operand) {return boost::any_cast<dSpaceID>(&operand);};
	bool is_body(const boost::any& operand) {return boost::any_cast<dBodyID>(&operand);};
	bool is_dtridata(const boost::any& operand) {return boost::any_cast<dTriMeshDataID>(&operand);};
	bool is_ltridata(const boost::any& operand) {return boost::any_cast<TriMeshDataStorage>(&operand);};
	
	unsigned int mNumObjs;
	MapIDObj mIDObjMap;
};

/*static void ResizeMatlabMatrix(mxArray* A, int m, int n)
{
	// Get pointer to data
	double* ptr = mxGetPr(A);

	// Resize M and N
	mxSetM(A,m);
	mxSetN(A,n);

	// Reallocate
	double* newptr = (double*)mxRealloc(ptr, m * n * sizeof(*ptr));
	mxSetPr(A, newptr);
	mxFree(ptr);

	for (int n = 0; n < 6*3; ++n) newptr[n] = 0.0;
};*/

#endif