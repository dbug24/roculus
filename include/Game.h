#ifndef _GAME_H_
#define _GAME_H_

#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <WayPoint.h>
#include <GameCFGParser.h>
#include <Room.h>
#include <GameObject.h>
#include <Door.h>
#include <Key.h>
#include <Treasure.h>
#include <Lock.h>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>

using namespace Ogre;

/** \brief Singleton pattern class to manage the demo-game.
 * This class manages the demo-game and is responsible for all related objects, their (repeatable) initialization and the game process.
 */
class Game {
  protected:
	Game() : initiated(false) {}
	/**< Default constructor, not to be used outside class scope. */
	~Game();
	/**< Default destructor. */
	Game(const Game&);
	/**< Copy constructor, hidden due to singleton pattern. */
	Game& operator=(const Game&);
	/**< Copy operator, hidden due to singleton pattern. */
	bool initiated; /**< Marks if the game was successfully initiated for the first time. */
	
	SceneManager* mSceneMgr;			/**< The scene manager (used for object creation). */
	SceneNode 	*marker, 				/**< The waypoint marker (green flagg) that will jump between possible navigation targets. */
				*persMarker;			/**< The waypoint marker (red flagg) that will stay on the current navigation target. */
	std::vector<WayPoint*> wayPoints;	/**< Vector of all waypoints. */
	std::vector<Room*> rooms;			/**< Vector of all rooms. */
	std::vector<Room*> corridors;		/**< Vector of all corridors. */
	std::vector<GameObject*> gameObjects;	/**< Vector of all GameObjects. */
	WayPoint *select;	/**< Waypoint to which the cursor is closest (with some maximum distance threshold). */
	WayPoint *initWP;	/**< Currently not in use, but generally the waypoint the robot should navigate to, if the game is restarted. The code for this should be commented-in in BaseApplication. */
	Real distMin;		/**< Internal calculus, global to avoid continuous memory alloc.: current found min. distance in the sorting algorithm. */
	Real distance;		/**< Internal calculus, global to avoid continuous memory alloc.: distance for the current iteration of the sorting algorithm. */
	GameState state;	/**< The current state of the game. */
	Door *door;			/**< The door in the game (there is just one). */
	Treasure *treasure;	/**< The treasure in the game (there is just one). */
	bool running;		/**< Is the game initialized and started? */
	
	Room *getRndRoom(std::set<int> &roomNRs);
	/**< Utility function: get a random room from the set of all rooms and erase that room number/index from the set. */
	
  public:
	static Game &getInstance();
	/**< Get the single instance of the Game class. */
  
	void init(SceneManager*);
	/**< Initialize the Game class. Has to be called before the singleton can be used with getInstance(). */
	WayPoint* getWPById(int);
	/**< Get a waypoint using its ID as identifier. */
	WayPoint* getWPByName(const String&);
	/**< Get a waypoint using its name as identifier. String search is not very performant -> use with care. (Not EACH frame!!!). */
	std::string getInitWP();
	/**< Get the waypoint the game shall be started at (robot location). */
	String highlightClosestWP(const Vector3&);
	/**< Find the closest waypoint and return its name.*/
	String getState();
	/**< Return the GameState. */
	void placePersistentMarker(const String&);
	/**< Place the persistent marker (for the navigation target) on the waypoint specified by name. */
	void print();
	/**< Print the configuration of the game (waypoints, corridors, rooms) on the console. Useful for debugging. */
	
	void startGameSession();
	/**< Start a new game (reinitialize). */ 
	GameState frameEventQueued(int);
	/**< Process the game. The GameState is known internally, the parameter is the current waypoint ID of the robot. */
	bool isRunning();
	/**< Is the game running? */
};

#endif
