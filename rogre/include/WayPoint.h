#ifndef _WAYPOINT_H_
#define _WAYPOINT_H_

#include <OgreString.h>
#include <OgreVector3.h>
#include <boost/lexical_cast.hpp>

using namespace Ogre; 

enum WayPoint_Role {WP_ROLE_NONE,
					WP_ROLE_TREASURE,
					WP_ROLE_KEY,
					WP_ROLE_DOOR};

class WayPoint {
  protected:
	int id;
	String name;
	Vector3 pos;
	WayPoint_Role role;
	bool accessible;
  public:
	WayPoint(int);
	WayPoint(int, Vector3, WayPoint_Role);
	String getName();
	Vector3 getPosition();
	int getId();
	WayPoint_Role getRole();
	void setRole(WayPoint_Role role);
	void setPosition(Vector3 pos);
	bool isAccessible();
	void setAccessibility(bool);
};

#endif
