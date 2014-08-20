#ifndef _GAME_OBJ_H_
#define _GAME_OBJ_H_

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h> //Not needed here,but in every child-class
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include <GameDefinitions.h>
#include <WayPoint.h>
#include <Room.h>
using namespace Ogre;

/** \brief Describes a generic game object.
 * This class is an abstract description of a game object. It give some default functionality and
 * is otherwise used to inherit the properties to its child classes: Door, Key, Treasure and Lock.
 */
class GameObject {
  protected:
	GameObject() {  }
	/**< Default constructor. Not to be used outside class scope.*/
	SceneNode *myNode;
	/**< Each GameObject has its own SceneNode.*/
	WayPoint *place;
	/**< Remember the waypoint this object is placed at.*/
	WayPoint *trigger;
	/**< The waypoint this object responds to (the one that causes the GameState to change).*/
	Room *room;
	/**< The room this object is associated with.*/
	bool initialized;
	/**< Remember if this object was initialized with the init(...) method.*/
	GameObjectType type;
	/**< Type of this object. Set in the child class.*/
  public:
	GameObject(SceneManager *mSceneMgr) : initialized(false), type(GO_GENERIC) {
		myNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		std::srand(time(NULL));
	} /**< Do some standard initialization, needed by all object types.*/
	
	void placeObjectOnWP(WayPoint *location) { 
		place = location;
		myNode->setPosition(place->getPosition());
		myNode->setVisible(true);
	} /**< Places the object on the specified WayPoint and sets the scene node's visibility to true.*/
	
	WayPoint *getTrigger() { return trigger; }
	/**< Return the triggering WayPoint.*/
	
	WayPoint *getWP() const { return place; }
	/**< Return the location of the object.*/
	
	GameObjectType getType() { return type; }
	/**< Return the type/use of this object.*/
	
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs) { 
		return gs; 
	} /**< Process the GameState and the current robot WayPoint to derive the next GameState.*/
	
	void setVisible(bool val) { myNode->setVisible(val); }						/**< Sets the scene node visibility.*/
	virtual void init(Room *room) { this->room = room; initialized = true; }	/**< (Re)initializes the GameObject and the specified Room for a new game session.*/
	void resetInit() { initialized = false; }									/**< Make the object ready for reinitialization. Has to be called before init every time after the first.*/
	bool isInitialized() { return initialized; }								/**< Return if the object was initialized with init(...).*/
};

#endif
