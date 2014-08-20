#ifndef _KEY_H_
#define _KEY_H_

#include <GameObject.h>
using namespace Ogre;

/** \brief Represents a Key in a Game.
 * This class is a more detailed description of the GameObject Key. It
 * specifies how to display and what to do with a key.
 */
class Key : public GameObject {
  protected:
    Key();		/**< Default constructor. Not to be used outside class scope.*/
    bool found;	/**< Was this key found, or not?*/
  public:
	Key(SceneManager*);
	/**< Initializes the key object and sets it up for display.*/
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs);
	/**< Reimplements GameObject::frameEventQueued(...). Processes the current robot WayPoint and GameState to derive if the next state is effected by this key.*/
	virtual void init(Room *room);
	/**< Reinitializes the object for a new game session.*/
};

#endif
