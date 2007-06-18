#ifndef OBMPL_BODY_H
#define OBMPL_BODY_H

#include <map>
#include <string>

namespace OBMPL {

	using namespace std;

	/**
	 *	Create a convex PhysX object from a point cloud.
	 *	Creates a PhysX object from a point cloud.
	 *	@param cloud a array of NxVec3 vectors defining the point cloud.
	 *	@param pos a NxVec3 position vector to the centroid of the point cloud.
	 *	@see Related_function()
	 *	@return A pointer to a NxActor object
	 */
	/*NxActor* CreateConvexObjectComputeHull(const NxVec3& cloud, const NxVec3& boxDim, const NxReal density)
    {
        NxActorDesc actorDesc;
        NxBodyDesc bodyDesc;

        // Compute hull
        NxVec3 verts[8] =
        { 
            NxVec3(-boxDim.x,-boxDim.y,-boxDim.z),
            NxVec3(boxDim.x,-boxDim.y,-boxDim.z),
            NxVec3(-boxDim.x,boxDim.y,-boxDim.z),
            NxVec3(boxDim.x,boxDim.y,-boxDim.z),
            NxVec3(-boxDim.x,-boxDim.y,boxDim.z),
            NxVec3(boxDim.x,-boxDim.y,boxDim.z),
            NxVec3(-boxDim.x,boxDim.y,boxDim.z),
            NxVec3(boxDim.x,boxDim.y,boxDim.z)
        };

        // Create descriptor for convex mesh
        NxConvexMeshDesc convexDesc;
        convexDesc.numVertices = 8;
        convexDesc.pointStrideBytes = sizeof(NxVec3);
        convexDesc.points = verts;
        convexDesc.flags = NX_CF_COMPUTE_CONVEX;

        NxConvexShapeDesc convexShapeDesc;
        convexShapeDesc.localPose.t = NxVec3(0,boxDim.y,0);

        NxInitCooking();
        if (0)
        {
            // Cooking from file
    #ifndef LINUX
            bool status = NxCookConvexMesh(convexDesc, UserStream("c:\\tmp.bin", false));
            convexShapeDesc.meshData = gPhysicsSDK->createConvexMesh(UserStream("c:\\tmp.bin", true));
    #else
            printf("Linux does not behave well with UserStreams, use MemorBuffers instead\n");
            exit(1);
    #endif
        }
        else
        {
            // Cooking from memory
            MemoryWriteBuffer buf;
            bool status = NxCookConvexMesh(convexDesc, buf);
            convexShapeDesc.meshData = gPhysicsSDK->createConvexMesh(MemoryReadBuffer(buf.data));
        }

        if (convexShapeDesc.meshData)
        {
            NxActorDesc actorDesc;
            actorDesc.shapes.pushBack(&convexShapeDesc);
            if (density)
            {
                actorDesc.body = &bodyDesc;
                actorDesc.density = density;
            }
            else
            {
                actorDesc.body = NULL;
            }
            actorDesc.globalPose.t = pos;
            return gScene->createActor(actorDesc);
    //      gPhysicsSDK->releaseTriangleMesh(*convexShapeDesc.meshData);
        }

        return NULL;
    }*/

	/**	
	 *	The Body class. The concept here is to encapsulate both the dynamic body description 
	 *	(PhysX's nxActor class) and its associate render object (OGRE's Entity class).
	 */
	class Body 
	{
	public:
		typedef map<string,string> MeshData;
		typedef MeshData::iterator MeshDataIterator;

		Body(const string& n, Vector3& p, Quaternion& q) : mName(n), mInitPos(p), mInitOrientation(q)  {};
		
		void addMesh(const string& id, const string& filename)
		{
			mMeshNames[id] = filename;
		}

		int numMeshes()
		{
			return mMeshNames.size();
		}
		
		MeshData mMeshNames;
		string mName;

		Ogre::Vector3 mInitPos;
		Ogre::Quaternion mInitOrientation;
	};

};

#endif