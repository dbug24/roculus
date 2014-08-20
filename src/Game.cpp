#include <Game.h>
using namespace Ogre;

Game &Game::getInstance() {
	// Singleton pattern, NOTE: init must be called once before getInstance()
	static Game instance;
	return instance;
}

void Game::init(SceneManager* mSceneMgr) {
	if (!initiated) {
		state = GS_START;
		running = false;
		this->mSceneMgr = mSceneMgr;
		
		/* initialize the waypoints with dummy values
		 * the real parameters will be transmitted from ROS
		 */
		for (int i=0;i<GameCFGParser::getInstance().getNrWayPoints();i++) {
			wayPoints.push_back(new WayPoint(i, mSceneMgr,
				Vector3::ZERO, Quaternion::IDENTITY, WP_ROLE_NONE));
			LogManager::getSingletonPtr()->logMessage(wayPoints[i]->toString() + " added to Game.");
		}
		
		// instanciate the waypoint markers (green flaggs)
		marker = mSceneMgr->getRootSceneNode()->createChildSceneNode("Game_WayPointMarker");
		Entity *wpEnt = mSceneMgr->createEntity("Cylinder.mesh");
		wpEnt->setMaterialName("roculus3D/WayPointMarker");
		marker->attachObject(wpEnt);
		persMarker = mSceneMgr->getRootSceneNode()->createChildSceneNode("Game_WayPointPersistentMarker");
		wpEnt = mSceneMgr->createEntity("Cylinder.mesh");
		wpEnt->setMaterialName("roculus3D/WayPointPersistentMarker");
		persMarker->attachObject(wpEnt);
		persMarker->setVisible(false);
		
		// create the rooms in the game
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
		
		// create the corridors in the game
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
		
		initWP = getWPByName(GameCFGParser::getInstance().getInitNode());
		
		// create all GameObjects
		door = new Door(mSceneMgr, GameCFGParser::getInstance().getNrKeys());
		treasure = new Treasure(mSceneMgr);
		gameObjects.push_back(door);
		gameObjects.push_back(treasure);
		for (int i=0;i<GameCFGParser::getInstance().getNrKeys();i++) {
			gameObjects.push_back(new Key(mSceneMgr));
			gameObjects.push_back(new Lock(mSceneMgr, i));
		}
		
		if (corridors.size() != GameCFGParser::getInstance().getNrCorridors() || rooms.size() != GameCFGParser::getInstance().getNrRooms())
			std::cerr << "There seems to be a mismatch between game.cfg and parsed game structure." << std::endl;
		initiated = true;
	}
}

Room *Game::getRndRoom(std::set<int> &roomNRs) {
	// get a random room from the set and remove its index
	int index = -1;
	do {
		index = std::rand() % rooms.size();
	} while (0 == roomNRs.count(index) || roomNRs.size() == 0);
	roomNRs.erase(index);
	return rooms[index];
}

void Game::startGameSession() {
	running = false;
	state = GS_START;
	
	std::srand(time(NULL));
	std::set<int> roomNRs;
	
	// fill up the set with all room indices	
	for (int i=0;i<rooms.size();i++) {
		rooms[i]->unlock();
		roomNRs.insert(i);
	}
	
	// make all GameObjects resetable
	for (int i=0;i<gameObjects.size();i++) {
		gameObjects[i]->resetInit();
	}
	
	// select the treasure room
	Room *treasureRoom = getRndRoom(roomNRs);
	
	// reinitialize all GameObjects
	for (int i=0;i<gameObjects.size();i++) {
		switch (gameObjects[i]->getType()) {
			case GO_DOOR: 
			case GO_TREASURE:
			case GO_LOCK: gameObjects[i]->init(treasureRoom); break;
			case GO_KEY: gameObjects[i]->init(getRndRoom(roomNRs)); break;
			default: std::cerr << "Could not start new game: undetermined Room Type." << std::endl;
		}
	}
	
	running = true;
}

GameState Game::frameEventQueued(int currentWPId) {
	// forward the id and the game state to all game objects
	// determine the following state
	/* This is not the most reliable way to build the game engine,
	 * but it works for our linear game. Note however that it is sensible
	 * to the order of the elements in the gameObjects vector.
	 */
	GameState next_state;
	for (int i=0;i<gameObjects.size();i++) {
		next_state = gameObjects[i]->frameEventQueued(wayPoints[currentWPId], state);
		if (next_state != state) break;
	}
	state = next_state;
	return state;
}

Game::~Game() {
	// clean up
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
	// getter
	if (id >= 0 && id < wayPoints.size())
		return wayPoints[id];
}

WayPoint* Game::getWPByName(const String& name) {
	// getter
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
	// simple search algorithm to find the closest waypoint
	for (int i=0;i<wayPoints.size();i++) {
		if (!wayPoints[i]->isAccessible()) continue;
		const Vector3& wp = wayPoints[i]->getPosition();
		distance = (pos.x-wp.x)*(pos.x-wp.x) + (pos.z-wp.z)*(pos.z-wp.z);
		if (distance < distMin) {
			distMin = distance;
			select = wayPoints[i];
		}
	}
	
	// only select it, if the distance is within reasonable limits
	if (select && distMin < 1.0f) {
		marker->setPosition(select->getPosition());
		marker->setVisible(true);
		//~ LogManager::getSingletonPtr()->logMessage(select->toString() + " at sq-distance: " + StringConverter::toString(distance));
		return select->getName();
	}
	
	// if the distance was too large
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
	// resolve the GameState as a Ogre::String and return it
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
	// Since the string based search is not very performant, use this method with care
	persMarker->setPosition(getWPByName(name)->getPosition());
	persMarker->setVisible(true);
}

std::string Game::getInitWP() {
	return initWP->getName();
}
