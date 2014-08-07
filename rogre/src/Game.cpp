#include <Game.h>
using namespace Ogre;

Game::Game(SceneManager* mSceneMgr) : lastWPId(-1), state(GS_START), running(false) {
	this->mSceneMgr = mSceneMgr;	
	for (int i=0;i<GameCFGParser::getInstance().getNrWayPoints();i++) {
		wayPoints.push_back(new WayPoint(i, mSceneMgr,
			Vector3::ZERO, Quaternion::IDENTITY, WP_ROLE_NONE));
		LogManager::getSingletonPtr()->logMessage(wayPoints[i]->toString() + " added to Game.");
	}
	
	marker = mSceneMgr->getRootSceneNode()->createChildSceneNode("Game_WayPointMarker");
	Entity *wpEnt = mSceneMgr->createEntity("Cylinder.mesh");
	wpEnt->setMaterialName("roculus3D/WayPointMarker");
	marker->attachObject(wpEnt);
	persMarker = mSceneMgr->getRootSceneNode()->createChildSceneNode("Game_WayPointPersistentMarker");
	wpEnt = mSceneMgr->createEntity("Cylinder.mesh");
	wpEnt->setMaterialName("roculus3D/WayPointPersistentMarker");
	persMarker->attachObject(wpEnt);
	persMarker->setVisible(false);
	
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
	door = new Door(mSceneMgr, 2);
	treasure = new Treasure(mSceneMgr);
	gameObjects.push_back(door);
	gameObjects.push_back(treasure);
	for (int i=0;i<2;i++) {
		gameObjects.push_back(new Key(mSceneMgr));
	}
	
	if (corridors.size() != GameCFGParser::getInstance().getNrCorridors() || rooms.size() != GameCFGParser::getInstance().getNrRooms())
		std::cerr << "There seems to be a mismatch between game.cfg and parsed game structure." << std::endl;
}

void Game::startGameSession() {
	running = false;
	
	std::srand(time(NULL));
	std::set<int> roomNRs;
	int index = -1;
	
	state = GS_START;
	for (int i=0;i<gameObjects.size();i++) {
		gameObjects[i]->resetInit();
		roomNRs.insert(i);
	}
	
	do {
		index = std::rand() % rooms.size();
	} while (0 == roomNRs.count(index) || roomNRs.size() == 0);
	roomNRs.erase(index);
	door->init(rooms[index]);
	treasure->init(rooms[index]);
	
	
	for (int i=0;i<gameObjects.size();i++) {
		if (gameObjects[i]->isInitialized()) continue;
		do {
			index = std::rand() % rooms.size();
		} while (0 == roomNRs.count(index) || roomNRs.size() == 0);
		roomNRs.erase(index);
		gameObjects[i]->init(rooms[index]);
	}
	
	running = true;
}

GameState Game::frameEventQueued(int currentWPId) {
	if (currentWPId != lastWPId) {
		lastWPId = currentWPId;
		GameState next_state;
		for (int i=0;i<gameObjects.size();i++) {
			next_state = gameObjects[i]->frameEventQueued(wayPoints[currentWPId], state);
		}
		state = next_state;
	}
	return state;
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
	for (int i=0;i<corridors.size();i++) {
		if (corridors[i]) {
			delete corridors[i];
			corridors[i] = NULL;
		}
	}
	for (int i=0;i<gameObjects.size();i++) {
		if (gameObjects[i]) {
			delete gameObjects[i];
			gameObjects[i] = NULL;
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

String Game::highlightClosestWP(const Vector3& pos) {
	select = NULL;
	distMin = 900000.0f;
	distance = distMin;
	for (int i=0;i<wayPoints.size();i++) {
		if (!wayPoints[i]->isAccessible()) continue;
		const Vector3& wp = wayPoints[i]->getPosition();
		distance = (pos.x-wp.x)*(pos.x-wp.x) + (pos.z-wp.z)*(pos.z-wp.z);
		if (distance < distMin) {
			distMin = distance;
			select = wayPoints[i];
		}
	}
	
	if (select && distMin < 1.0f) {
		marker->setPosition(select->getPosition());
		marker->setVisible(true);
		//~ LogManager::getSingletonPtr()->logMessage(select->toString() + " at sq-distance: " + StringConverter::toString(distance));
		return select->getName();
	}
	
	marker->setVisible(false);
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

String Game::getState() {
	if (state == GS_START) return String("START"); else
	if (state == GS_KEY_1) return String("KEY_1"); else
	if (state == GS_KEY_2) return String("KEY_2"); else
	if (state == GS_KEY_3) return String("KEY_3"); else
	if (state == GS_KEY_4) return String("KEY_4"); else
	if (state == GS_DOOR_OPEN) return String("DOOR_OPEN"); else
	if (state == GS_FOUND_TREASURE) return String("FOUND_TREASURE");
	std::cout << "getState()" << std::endl;
}

bool Game::isRunning() {
	return running;
}

void Game::placePersistentMarker(const String& name) {
	persMarker->setPosition(getWPByName(name)->getPosition());
	persMarker->setVisible(true);
}
