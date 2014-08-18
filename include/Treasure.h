#ifndef _TREASURE_H_
#define _TREASURE_H_

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <GameObject.h>
#include <OgreParticleSystem.h>
using namespace Ogre;

class Treasure : public GameObject {
  protected:
    Treasure();
    ParticleSystem *gold,*fireworks;
  public:
	Treasure(SceneManager*);
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs); 
	virtual void init(Room *room);
};

#endif
