#ifndef _TREASURE_H_
#define _TREASURE_H_

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <GameObject.h>
#include <OgreParticleSystem.h>
using namespace Ogre;

/** \brief Describes the appearance and behaviour of a Treasure in the Game.
 * This class is similar to the other GameObjects.
 */
class Treasure : public GameObject {
  protected:
    Treasure();
    /**< Default constructor. Not to be used outside class scope.*/
    ParticleSystem 	*gold,		/**< Sparkling particle effect as eye-candy when the treasure is found.*/
					*fireworks;	/**< Fireworks as eye-candy, when the treasure is found.*/
  public:
	Treasure(SceneManager*);
	/**< Initialize the Treasure object with the scene manager for object creation.*/
	virtual GameState frameEventQueued(WayPoint* currentWP, GameState gs); 
	/**< Process the current robot WayPoint and GameState for the Treasure behaviour.*/
	virtual void init(Room *room);
	/**< (Re)initialize the treasure for a new game session.*/
};

#endif
