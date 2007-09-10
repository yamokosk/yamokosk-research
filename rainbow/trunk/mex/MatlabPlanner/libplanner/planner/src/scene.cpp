#include "planner\scene.h"

Scene::Scene()
{
	dInitODE();
	world_ = dWorldCreate();
}

Scene::~Scene()
{
	dWorldDestroy(world_);
	dCloseODE();
}

Body Scene::getBody(const std::string& name)
{
	BodyMap::iterator it;
	it = bodyContainer_.find(name);
	if (it != bodyContainer_.end()) {
		return it->second;	
	} else {
		return Body();
	}
}

void Scene::addBody(std::string name, std::string parent)
{
	// No checking if body already exists!
	Body b;
	b.id_ = dBodyCreate(world_);
	b.parent_ = parent;
	bodyContainer_[name] = b;
}