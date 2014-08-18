#ifndef _LOCK_H_
#define _LOCK_H_

#include <GameObject.h>
using namespace Ogre;

class Lock : public GameObject {
  protected:
    Lock() { }
    GameState react;
    Entity *ent;
    int idx;
    bool locked;
  public:
	Lock(SceneManager*, int);
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs);
	virtual void init(Room *room);
};

#endif
