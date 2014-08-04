#ifndef _GAME_H_
#define _GAME_H_

#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <WayPoint.h>
using namespace Ogre;

class Game {
  protected:
	SceneManager* mSceneMgr;
	SceneNode* cursor;
	SceneNode* marker;
	std::vector<WayPoint*> wayPoints;
	WayPoint *select;
	Real distMin;
	Real distance;
	Vector3 markerPos;
  public:
	Game(SceneManager*, int);
	//~ Game(SceneManager*, const std::string&);
	~Game();
	WayPoint* getWPById(int id);
	WayPoint* getWPByName(const String& name);
	String highlightClosestWP(Vector3 pos);
};

#endif
