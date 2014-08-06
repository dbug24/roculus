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

using namespace Ogre;

class Game {
  protected:
	Game();
	SceneManager* mSceneMgr;
	SceneNode* marker;
	std::vector<WayPoint*> wayPoints;
	std::vector<Room*> rooms;
	std::vector<Room*> corridors;
	std::vector<GameObject*> gameObjects;
	WayPoint *select;
	Real distMin;
	Real distance;
	Vector3 markerPos;
	GameState state;
	int lastWPId;
	
	
  public:
	Game(SceneManager*);
	~Game();
	WayPoint* getWPById(int id);
	WayPoint* getWPByName(const String& name);
	String highlightClosestWP(Vector3 pos);
	void print();
	
	void startGameSession();
	void frameEventQueued(int);
};

#endif
