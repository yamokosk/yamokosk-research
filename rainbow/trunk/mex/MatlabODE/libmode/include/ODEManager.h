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


#define MEMCPY( dest, src, n ) \
   for( int i = 0; i <= (n); ++i ) (dest[i]) = (src[i]);

using namespace std;

/*template<class dObject>
class ObjectManager
{
public:
	ObjectManager(boost::function<void (dObject)> f) : mNumObjs(1), mfDestroy(f) {};
	~ObjectManager() 
	{
		while( !mObjIDMap.empty() )	{
			map<int,dObject>::iterator it = mObjIDMap.begin();
			mfDestroy(it->second);
			mObjIDMap.erase(it->first);
		}
	};

	unsigned int add(dObject& ptr)
	{
		mNumObjs++;
		mObjIDMap[mNumObjs] = ptr;
		return mNumObjs;
	};

	bool get(unsigned int id, dObject& dID)
	{
		map<int,dObject>::iterator it = mObjIDMap.find(id);
		if (it != mObjIDMap.end()) {
			dID = it->second;
			return true;
		} else {
			return false;
		}
	};

	bool getID(int& ID, dObject& dID)
	{
		map<int,dObject>::iterator it;
		for (it = mObjIDMap.begin(); it != mObjIDMap.end(); ++it)
		{
			if (dID == it->second) {ID = it->first; return true;}
		}
		ID = 0;
		return false;
	};

	bool destroy(int id)
	{
		map<int,dObject>::iterator it = mObjIDMap.find(id);
		if (it != mObjIDMap.end()) {
			mfDestroy(it->second);
			mObjIDMap.erase(it->first);
			return true;
		} else {
			return false;
		}
	};

	int size() {return mObjIDMap.size();};
	
	vector<dObject> getAllObjs()
	{
		vector<dObject> objs;
		map<int, dObject>::iterator it;
		for (it = mObjIDMap.begin(); it != mObjIDMap.end(); ++it)
		{
			objs.push_back( it->second );
		}
		return objs;
	};

	vector<int> getAllIDs()
	{
		vector<int> IDs;
		map<int, dObject>::iterator it;
		for (it = mObjIDMap.begin(); it != mObjIDMap.end(); ++it)
		{
			IDs.push_back( it->first );
		}
		return IDs;
	};

protected:
	map<int,dObject> mObjIDMap;
	unsigned int mNumObjs;
	boost::function<void (dObject)> mfDestroy;
};

class ODEManager
{
public:
	ODEManager() : 
		mGeomManager(dGeomDestroy),
		mSpaceManager(dSpaceDestroy),
		mBodyManager(dBodyDestroy) {mWorld = dWorldCreate();}
	~ODEManager() {dWorldDestroy(mWorld);}

	bool get(int nID, dGeomID& dID) {return mGeomManager.get(nID, dID);};
	bool get(int nID, dSpaceID& dID) {return mSpaceManager.get(nID, dID);};
	bool get(int nID, dBodyID& dID) {return mBodyManager.get(nID, dID);};
	
	bool destroyGeom(unsigned int nID) {return mGeomManager.destroy(nID);};
	bool destroySpace(unsigned int SID) 
	{
		dSpaceID dSID = NULL;
		if (!get(SID, dSID)) return false;
		
		if (dSpaceGetCleanup(dSID)) // dSpaceDestroy will destroy all Geoms in it
		{
			// Get all geoms in this space and destroy them as well
			vector<int> allGeomIDs = getAllGeomIDs();
			for (int n=0; n < allGeomIDs.size(); ++n) {
				dGeomID dGID = NULL; get(allGeomIDs[n], dGID);
				if (dSpaceQuery(dSID, dGID)) destroyGeom(allGeomIDs[n]);
			}
		} 
		
		return mSpaceManager.destroy(SID);
	};
	bool destroyBody(unsigned int nID) {return mBodyManager.destroy(nID);};
	
	int add(dGeomID dID) {return mGeomManager.add(dID);};
	int add(dSpaceID dID) {return mSpaceManager.add(dID);};
	int add(dBodyID dID) {return mBodyManager.add(dID);};
	
	int numGeoms() {return mGeomManager.size();};
	int numSpaces() {return mSpaceManager.size();};
	int numBodies() {return mBodyManager.size();};

	bool getID(int& ID, dGeomID& dID) {return mGeomManager.id(ID, dID);};
	bool getID(int& ID, dSpaceID& dID) {return mSpaceManager.id(ID, dID);};
	bool getID(int& ID, dBodyID& dID) {return mBodyManager.id(ID, dID);};

	//list<dBodyID> getAllBodyObjs() {return mBodyManager.getAllObjs();}
	//list<dSpaceID> getAllSpaceObjs() {return mSpaceManager.getAllObjs();};
	//list<dGeomID> getAllGeomObjs() {return mGeomManager.getAllObjs();};

	vector<int> getAllBodyIDs() {return mBodyManager.getAllIDs();};
	vector<int> getAllSpaceIDs() {return mSpaceManager.getAllIDs();};
	vector<int> getAllGeomIDs() {return mGeomManager.getAllIDs();};

protected:
	ObjectManager<dGeomID> mGeomManager;
	ObjectManager<dSpaceID> mSpaceManager;
	ObjectManager<dBodyID> mBodyManager;
	dWorldID mWorld;
};*/

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

	ODEManager() : mNumObjs(0) {mWorld = dWorldCreate();}
	~ODEManager() 
	{
		while( !mIDObjMap.empty() ) {
			MapIDObjIterator it = mIDObjMap.begin();
			destroy( it->first );
		}
		dWorldDestroy(mWorld);
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
				vector<int> ag = this->all<dGeomID>();
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

	template<class dObject>
	vector<int> all()
	{
		vector<int> out;
		for (MapIDObjIterator it = mIDObjMap.begin(); it !=mIDObjMap.end(); ++it) {
			if ( boost::any_cast<dObject>(it->second) ) out.push_back(it->first);
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