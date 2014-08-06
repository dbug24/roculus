#ifndef _TREASURE_H_
#define _TREASURE_H_

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <GameObject.h>
using namespace Ogre;

class Treasure : public GameObject {
  protected:
    Treasure();
  public:
	Treasure(SceneManager*);
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs); 
};

#endif
