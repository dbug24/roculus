#ifndef _GAME_H_
#define _GAME_H_

#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <WayPoint.h>
#include <GameCFGParser.h>
#include <Room.h>

using namespace Ogre;

class Game {
  protected:
	Game();
	SceneManager* mSceneMgr;
	SceneNode* marker;
	std::vector<WayPoint*> wayPoints;
	std::vector<Room*> rooms;
	std::vector<Room*> corridors;
	WayPoint *select;
	Real distMin;
	Real distance;
	Vector3 markerPos;
  public:
	Game(SceneManager*);
	~Game();
	WayPoint* getWPById(int id);
	WayPoint* getWPByName(const String& name);
	String highlightClosestWP(Vector3 pos);
	void print();
};

#endif
