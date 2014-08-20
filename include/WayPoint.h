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

/** \brief Defines a WayPoint for the Game.
 * Waypoints are possible navigation targets and are used to place GameObjects, like Keys and Locks. The list of WayPoints is held by the Game class, but they are read
 * from other classes quite often, usually passed as a pointer or reference. A WayPoint can be made inaccessible for the selection by the player setting the accessible flag.*/
class WayPoint {
  protected:
	boost::mutex WAYPOINT_MUTEX;/**< Mutex to ensure unique access to the waypoints. (Haven't really tested if this makes any difference).*/
	int id;						/**< ID of this waypoint.*/
	SceneManager *mSceneMgr;	/**< Scene manager, to create the nodes/entities for display.*/
	SceneNode *wpSN;			/**< The scene node of this waypoint.*/
	Entity *wpEnt;				/**< The entity of this waypoint. Stored to be able to change the material (transparent if not accessible).*/	
	String name;				/**< The name of the waypoint. Comform with the ROS notation "WayPoint".append(#ID).*/
	Vector3 pos;				/**< The position of this waypoint.*/
	Quaternion ori;				/**< The orientation of this waypoint. If the waypoint is a navigation target, ROSIE will turn into this direction. If it is just a node in the path, it does not play a role.*/
	WayPoint_Role role;			/**< The role of this waypoint in the Game. (Not in use).*/
	bool accessible;			/**< Can this waypoint be targeted for navigation?*/
  public:
	WayPoint(int, SceneManager*, Vector3, Quaternion, WayPoint_Role);
	/**< Initialize a WayPoint with: (1) a unique ID, (2) the scene manager, (3) the position and (4) its role. */
	String getName();					/**< Returns the name of this waypoint.*/
	const Vector3& getPosition();		/**< Returns the position of this waypoint.*/
	const Quaternion& getOrientation();	/**< Returns the orientation of this waypoint.*/
	int getId();						/**< Returns the ID of this waypoint.*/
	WayPoint_Role getRole();			/**< Returns the role of this waypoint.*/
	void setRole(WayPoint_Role role);	/**< Sets the role of this waypoint.*/
	void setPosition(const Vector3& pos);	/**< Sets the position of this waypoint.*/
	void setOrientation(const Quaternion& orientation);	/**< Sets the orientation of this waypoint.*/
	bool isAccessible();				/**< Can this waypoint currently be a navigation target?*/
	void setAccessibility(bool);		/**< Set the accessibility of this waypoint (true means accessible).*/
	void setVisible(bool visible);		/**< Set the visibility of this waypoint. Forwarded to the scene node.*/
	std::string toString();				/**< Return a string with the parameters of this waypoint. Useful for debugging.*/
};

#endif
