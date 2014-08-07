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

class GameObject {
  protected:
	GameObject() {  }
	SceneNode *myNode;
	WayPoint *place;
	WayPoint *trigger;
	Room *room;
	bool initialized;
  public:
	GameObject(SceneManager *mSceneMgr) : initialized(false) {
		myNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		std::srand(time(NULL));
	}
	
	void placeObjectOnWP(WayPoint *location) { 
		place = location;
		myNode->setPosition(place->getPosition());
		myNode->setVisible(true);
	}
	
	void setTrigger(WayPoint* trigger) { this->trigger = trigger; }
	
	WayPoint *getWP() const { return place; }
	
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs) { 
		return gs; 
	}
	
	void setVisible(bool val) { myNode->setVisible(val); }
	virtual void init(Room *room) { this->room = room; initialized = true; }
	void resetInit() { initialized = false; }
	bool isInitialized() { return initialized; }
};

#endif
