/*
-----------------------------------------------------------------------------
This source file is part of OGRE
    (Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2006 Torus Knot Software Ltd
Also see acknowledgements in Readme.html

You may use this sample code for anything you like, it is not covered by the
LGPL like the rest of the engine.
-----------------------------------------------------------------------------
*/
/*
-----------------------------------------------------------------------------
Filename:    SkeletalAnimation.h
Description: Specialisation of OGRE's framework application to show the
             skeletal animation feature, including spline animation.
-----------------------------------------------------------------------------
*/


#include "ExampleApplication.h"
#include "body.h"

// PhysX includes
#undef random
#include "NxPhysics.h"

using namespace OBMPL;

// Event handler to animate
class NxOgreFrameListener : public ExampleFrameListener
{
public:
	NxOgreFrameListener(RenderWindow* win, Camera* cam, const std::string &debugText)
        : ExampleFrameListener(win, cam)
    {
		mDebugText = debugText;
    }

    bool frameStarted(const FrameEvent& evt)
    {
		if( ExampleFrameListener::frameStarted(evt) == false ) return false;

		/*Real inc = evt.timeSinceLastFrame * mAnimationSpeed[i]; 
		if ((mAnimState[i]->getTimePosition() + inc) >= mAnimChop)
		{
			// Get pointer to the PhysX SDK
			
			// Get results from gScene->simulate(gDeltaTime)
			while (!gScene->fetchResults(NX_RIGID_BODY_FINISHED, false));
			
			// Loop
			// Need to reposition the scene node origin since animation includes translation
			// Calculate as an offset to the end position, rotated by the
			// amount the animation turns the character
			Quaternion rot(mAnimationRotation, Vector3::UNIT_Y);
			Vector3 startoffset = mSceneNode[i]->getOrientation() * -mSneakStartOffset;
			Vector3 endoffset = mSneakEndOffset;
			Vector3 offset = rot * startoffset;
			Vector3 currEnd = mSceneNode[i]->getOrientation() * endoffset + mSceneNode[i]->getPosition();
			mSceneNode[i]->setPosition(currEnd + offset);
			mSceneNode[i]->rotate(rot);

			mAnimState[i]->setTimePosition((mAnimState[i]->getTimePosition() + inc) - mAnimChop);
		} else {
			mAnimState[i]->addTime(inc);
		}*/
        
        return true;
    }
};



class NxOgreApplication : public ExampleApplication
{

public:
	NxOgreApplication() : mPhysicsSDK(NULL), mScene(NULL) {}
	virtual ~NxOgreApplication()
	{
		ReleaseNx();
	}
protected:
	std::string mDebugText;

	NxPhysicsSDK* mPhysicsSDK;
	NxScene*  mScene;
	
    // Just override the mandatory create scene method
    void createScene(void)
    {
		// Create robot
		Entity* ent= NULL;
		SceneNode* node = NULL;

		// Link 1
		Body* linkOne = getLinkOne();
		node = mSceneMgr->getRootSceneNode()->createChildSceneNode( linkOne->mName );		
		for (Body::MeshDataIterator it = linkOne->mMeshNames.begin(); it != linkOne->mMeshNames.end(); ++it)
		{
			ent = mSceneMgr->createEntity( it->first, it->second );
			ent->setCastShadows( true );
			node->attachObject( ent );
		}
		node->translate( linkOne->mInitPos );
		node->rotate( linkOne->mInitOrientation );

		// Link 2
		Body* linkTwo = getLinkTwo();
		node = mSceneMgr->getSceneNode( linkOne->mName )->createChildSceneNode( linkTwo->mName );
		for (Body::MeshDataIterator it = linkTwo->mMeshNames.begin(); it != linkTwo->mMeshNames.end(); ++it)
		{
			ent = mSceneMgr->createEntity( it->first, it->second );
			ent->setCastShadows( true );
			node->attachObject( ent );
		}
		node->rotate( linkTwo->mInitOrientation );
		node->translate( linkTwo->mInitPos );
		
    }

    void createFrameListener(void)
    {
        mFrameListener = new NxOgreFrameListener(mWindow, mCamera, mDebugText);
        mRoot->addFrameListener(mFrameListener);
    }
	
	void chooseSceneManager(void)
    {
        // Create the SceneManager, in this case a generic one
		mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC, "ExampleSMInstance");
    }
	
	// Init PhysX engine
	void InitNx()
	{
		// Create the physics SDK
		mPhysicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);
		if (!mPhysicsSDK)  return;

		// Set the physics parameters
		mPhysicsSDK->setParameter(NX_SKIN_WIDTH, 0.01);

		// Set the debug visualization parameters
		mPhysicsSDK->setParameter(NX_VISUALIZATION_SCALE, 1);
		mPhysicsSDK->setParameter(NX_VISUALIZE_COLLISION_SHAPES, 1);
		mPhysicsSDK->setParameter(NX_VISUALIZE_ACTOR_AXES, 1);

		// Create the scene
		NxSceneDesc sceneDesc;
 		sceneDesc.simType				= NX_SIMULATION_HW;
		sceneDesc.gravity               = NxVec3(0,-9.8,0);
		mScene = mPhysicsSDK->createScene(sceneDesc);	
		if(!mScene){ 
			sceneDesc.simType				= NX_SIMULATION_SW; 
			mScene = mPhysicsSDK->createScene(sceneDesc);  
			if(!mScene) return;
		}

		// Create the default material
		NxMaterial* defaultMaterial = mScene->getMaterialFromIndex(0); 
		defaultMaterial->setRestitution(0.5);
		defaultMaterial->setStaticFriction(0.5);
		defaultMaterial->setDynamicFriction(0.5);

		// Create the objects in the scene
		//groundPlane = CreateGroundPlane();
		//box = CreateBox();

		// Initialize HUD
		//InitializeHUD();

		// Get the current time
		//UpdateTime();

		// Start the first frame of the simulation
		if (mScene)  StartPhysics();
	};

	void ReleaseNx()
	{
		if (mScene)
		{
			GetPhysicsResults();  // Make sure to fetchResults() before shutting down
			mPhysicsSDK->releaseScene(*mScene);
		}
		if (mPhysicsSDK)  mPhysicsSDK->release();
	};

	void ResetNx()
	{
		ReleaseNx();
		InitNx();
	};
	void StartPhysics()
	{
		// Start collision and dynamics for delta time since the last frame
		mScene->simulate(0.01);
		mScene->flushStream();
	}
	void GetPhysicsResults()
	{
		// Get results from gScene->simulate(gDeltaTime)
		while (!mScene->fetchResults(NX_RIGID_BODY_FINISHED, false));
	}
	Body* getLinkOne()
	{
		Body* b = new Body("arm1", Vector3(0,0,0), Quaternion(1,0,0,0));
		b->addMesh("arm1_1","arm1_1.mesh");
		b->addMesh("arm1_2","arm1_2.mesh");
		b->addMesh("arm1_3","arm1_3.mesh");
		b->addMesh("arm1_4","arm1_4.mesh");
		b->addMesh("arm1_5","arm1_5.mesh");
		b->addMesh("arm1_6","arm1_6.mesh");
		b->addMesh("arm1_7","arm1_7.mesh");

		return b;
	}

	Body* getLinkTwo()
	{
		Body* b = new Body("arm2", Vector3(0,0,.450), Quaternion(Degree(-90), Vector3(1,0,0)) );
		b->addMesh("arm2_1","arm2_1.mesh");
		b->addMesh("arm2_2","arm2_2.mesh");
		b->addMesh("arm2_3","arm2_3.mesh");
		b->addMesh("arm2_4","arm2_4.mesh");
		
		return b;
	}

};

