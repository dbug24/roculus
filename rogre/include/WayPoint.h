#ifndef _WAYPOINT_H_
#define _WAYPOINT_H_

#include <OgreString.h>
#include <OgreStringConverter.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <boost/lexical_cast.hpp>

using namespace Ogre; 

enum WayPoint_Role {WP_ROLE_NONE,
					WP_ROLE_TREASURE,
					WP_ROLE_KEY,
					WP_ROLE_DOOR};

class WayPoint {
  protected:
	int id;
	SceneNode* wpSN;
	String name;
	Vector3 pos;
	Quaternion ori;	
	WayPoint_Role role;
	bool accessible;
  public:
	WayPoint(int, SceneNode*, Vector3, Quaternion, WayPoint_Role);
	String getName();
	const Vector3& getPosition();
	const Quaternion& getOrientation();
	int getId();
	WayPoint_Role getRole();
	void setRole(WayPoint_Role role);
	void setPosition(const Vector3& pos);
	void setOrientation(const Quaternion& orientation);
	bool isAccessible();
	void setAccessibility(bool);
	SceneNode* getSceneNode();
	void setVisible(bool visible);
	std::string toString();
};

#endif
