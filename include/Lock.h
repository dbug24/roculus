#ifndef _LOCK_H_
#define _LOCK_H_

#include <GameObject.h>
using namespace Ogre;

/** \brief Describes a Lock in a Game.
 * This class specifies the details to display a lock and implements its behaviour.
 */
class Lock : public GameObject {
  protected:
    Lock() { }		/**< Default constructor. Not to be used outside class scope.*/
    GameState react;/**< The GameState this lock will respond to (open).*/
    Entity *ent;	/**< Remembers its entity in order to be able to change the material.*/
    int idx;		/**< Number of the key that triggers the lock.*/
    bool locked;	/**< Is this Lock locked, or open?*/
  public:
	Lock(SceneManager*, int);
	/**< Initialize the Lock with the scene manager (to set up the display) and the number of the key to respond to for the game logic.*/
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs);
	/**< Reimplements GameObject::frameEventQueued(...). Given the inputs, derive next GameState.*/
	virtual void init(Room *room);
	/**< Reinitialize the Lock for a new game.*/
};

#endif
