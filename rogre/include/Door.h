#ifndef _DOOR_H_
#define _DOOR_H_

#include <GameObject.h>
using namespace Ogre;

class Door : public GameObject {
  protected:
    Door();
    GameState keys;
    const Vector3 mask;
  public:
	Door(SceneManager*, int);
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs); 
	virtual void init(Room *room);
};

#endif

