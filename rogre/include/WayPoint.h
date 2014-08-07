#ifndef _WAYPOINT_H_
#define _WAYPOINT_H_

#include <OgreString.h>
#include <OgreStringConverter.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <boost/lexical_cast.hpp>
#include <GameDefinitions.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

using namespace Ogre; 

class WayPoint {
  protected:
	boost::mutex WAYPOINT_MUTEX;
	int id;
	SceneManager *mSceneMgr;
	SceneNode *wpSN;
	Entity *wpEnt;
	String name;
	Vector3 pos;
	Quaternion ori;	
	WayPoint_Role role;
	bool accessible;
  public:
	WayPoint(int, SceneManager*, Vector3, Quaternion, WayPoint_Role);
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
	void setVisible(bool visible);
	std::string toString();
};

#endif
