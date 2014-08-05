#include <Game.h>
using namespace Ogre;

Game::Game(SceneManager* mSceneMgr) {
	this->mSceneMgr = mSceneMgr;
	wayPoints.clear();
	Entity* wpEnt;
	for (int i=0;i<GameCFGParser::getInstance().getNrWayPoints();i++) {
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
	
	marker = mSceneMgr->getRootSceneNode()->createChildSceneNode("Game_WayPointMarker");
	wpEnt = mSceneMgr->createEntity("Cylinder.mesh");
	wpEnt->setMaterialName("roculus3D/WayPointMarker");
	marker->attachObject(wpEnt);
	
	// link WayPoints in list to WayPoints in game.cfg
	for (int i=0;i<GameCFGParser::getInstance().getNrRooms();i++) {
		rooms.push_back(new Room(i));
		std::string tmp_name("Room" + boost::lexical_cast<std::string>(i));
		rooms[i]->setDoor(wayPoints[GameCFGParser::getInstance().getDoor(tmp_name)]);
		rooms[i]->setDoorEvt(wayPoints[GameCFGParser::getInstance().getDoorEvt(tmp_name)]);
		std::vector<int> tmp_values(GameCFGParser::getInstance().getWPs(tmp_name));
		for (int j=0;j<tmp_values.size();j++) {
			rooms[i]->addRoomPoint(wayPoints[tmp_values[j]]);
		}
		tmp_values.clear();
		tmp_values = GameCFGParser::getInstance().getWPs2Use(tmp_name);
		for (int j=0;j<tmp_values.size();j++) {
			rooms[i]->addUsePoint(wayPoints[tmp_values[j]]);
		}
	}
	
	for (int i=0;i<GameCFGParser::getInstance().getNrCorridors();i++) {
		corridors.push_back(new Room(i));
		std::string tmp_name("Corridor" + boost::lexical_cast<std::string>(i));
		corridors[i]->setDoor(NULL);
		corridors[i]->setDoorEvt(NULL);
		std::vector<int> tmp_values(GameCFGParser::getInstance().getWPs(tmp_name));
		for (int j=0;j<tmp_values.size();j++) {
			corridors[i]->addRoomPoint(wayPoints[tmp_values[j]]);
		}
	}
	
	if (corridors.size() != GameCFGParser::getInstance().getNrCorridors() || rooms.size() != GameCFGParser::getInstance().getNrRooms())
		std::cerr << "There seems to be a mismatch between game.cfg and parsed game structure." << std::endl;
}

Game::~Game() {
	for (int i=0;i<wayPoints.size();i++) {
		if (wayPoints[i]) {
			delete wayPoints[i];
			wayPoints[i] = NULL;
		}
	}
	for (int i=0;i<rooms.size();i++) {
		if (rooms[i]) {
			delete rooms[i];
			rooms[i] = NULL;
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

void Game::print() {
	std::cout << "Corridors [] :" << std::endl;
	for (int i=0;i<corridors.size();i++) {
		corridors[i]->print();
	}
	std::cout << "Rooms [] :" << std::endl;
	for (int i=0;i<rooms.size();i++) {
		rooms[i]->print();
	}
}
