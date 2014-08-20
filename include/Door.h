#ifndef _DOOR_H_
#define _DOOR_H_

#include <GameObject.h>
using namespace Ogre;

/** \brief Represents a Door in the demo-game.
 * A door is a GameObject and takes care of the appearance, as well as the behaviour of a door.
 * */
class Door : public GameObject {
  protected:
    Door();
    /**< Default constructor may not be used outside class scope. */
    GameState keys; 		/**< Specifies in which GameState the Door event (to open/disappear) can be triggered. */
    const Vector3 mask;		/**< For internal calculation of the door position. */
  public:
	Door(SceneManager*, int);
	/**< Constructor: initialized the elements for appearance and some game parameters. Takes the scene manager (to set up the scene node and entities) and the number of keys that are placed in the game. */
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs); 
	/**< If the game is in the door event GameState and the robot reached the trigger. Make the room accessible and the door disappear. */
	virtual void init(Room *room);
	/**< (Re)Initialize the door. Place it according to the given room specifications, lock the room and make the virtual door visible again. */
};

#endif

