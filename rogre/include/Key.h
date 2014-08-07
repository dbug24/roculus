#ifndef _KEY_H_
#define _KEY_H_

#include <GameObject.h>
using namespace Ogre;

class Key : public GameObject {
  protected:
    Key();
    bool found;
  public:
	Key(SceneManager*);
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs);
	virtual void init(Room *room);
};

#endif
