#ifndef SCENE_HEADER_FILE
#define SCENE_HEADER_FILE

#include "ode/ode.h"
#include <string>
#include <map>

struct Space
{
	dSpaceID id_;
};

struct Body
{
	std::string parent_;
	dBodyID id_;
};

struct Geom
{
	dGeomID id_;
};

class Scene
{
	//typedef std::map< std::string, double > VariableMap;

	Scene();
	~Scene();

	//void build();
	//void update(const VariableMap& vm);

	Body getBody(const std::string& name);
	void addBody(std::string name, std::string parent);
	//bool collideSpace2(Space& s1, Space& s2);

private:
	typedef std::map<std::string, Body> BodyMap;
	//VariableMap variables_;
	//std::map< variable_name, body_name >;
	BodyMap bodyContainer_;
	dWorldID world_;
};


#endif