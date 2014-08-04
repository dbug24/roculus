#include <Game.h>
using namespace Ogre;

Game::Game(SceneManager* mSceneMgr, int numberOfWPs) {
	this->mSceneMgr = mSceneMgr;
	wayPoints.clear();
	Entity* wpEnt;
	for (int i=0;i<numberOfWPs;i++) {
		wayPoints.push_back(new WayPoint(i,
			mSceneMgr->getRootSceneNode()->createChildSceneNode(),
			Vector3::ZERO,
			Quaternion::IDENTITY,
			WP_ROLE_NONE));
		wpEnt = mSceneMgr->createEntity("Torus.001.mesh");
		wpEnt->setMaterialName("roculus3D/WayPoint");
		wayPoints[i]->getSceneNode()->attachObject(wpEnt);
		wayPoints[i]->setVisible(false);
		LogManager::getSingletonPtr()->logMessage(wayPoints[i]->toString() + " added to Game.");
	}
	
	cursor = NULL;
	marker = mSceneMgr->getRootSceneNode()->createChildSceneNode("Game_WayPointMarker");
	wpEnt = mSceneMgr->createEntity("Cylinder.mesh");
	wpEnt->setMaterialName("roculus3D/WayPointMarker");
	marker->attachObject(wpEnt);
	//~ marker->setVisible(false);
}

//~ Game::Game(SceneManager*, const std::string&) {
	//~ //nothing yet, parse the game.cfg
//~ }

Game::~Game() {
	for (int i=0;i<wayPoints.size();i++) {
		if (wayPoints[i]) {
			delete wayPoints[i];
			wayPoints[i] = NULL;
		}
	}
}

WayPoint* Game::getWPById(int id) {
	if (id >= 0 && id < wayPoints.size())
		return wayPoints[id];
}

WayPoint* Game::getWPByName(const String& name) {
	for (int i=0;i<wayPoints.size();i++) {
		if (0 == name.compare(wayPoints[i]->getName())) {
			return wayPoints[i];
		}
	}
	return NULL;
}

String Game::highlightClosestWP(Vector3 pos) {
	select = NULL;
	distMin = 900000.0f;
	distance = distMin;
	for (int i=0;i<wayPoints.size();i++) {
		distance = pos.squaredDistance(wayPoints[i]->getPosition());
		if (distance < distMin) {
			distMin = distance;
			select = wayPoints[i];
		}
	}
	
	if (select) {
		marker->setPosition(select->getPosition());
		LogManager::getSingletonPtr()->logMessage(select->toString() + " at sq-distance: " + StringConverter::toString(distance));
		return select->getName();
	}
	
	return StringUtil::BLANK;
}
