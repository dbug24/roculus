#include <Door.h>
using namespace Ogre;

// This should be straight forward...

Door::Door(SceneManager* mSceneMgr, int keys) : GameObject(mSceneMgr), mask(1.0f, 0.0f, 1.0f) {
	// Initialize all objects/parameters needed for the door
	Entity *ent = mSceneMgr->createEntity("Door.mesh");
	ent->setMaterialName("roculus3D/Game_Door");
	myNode->attachObject(ent);
	switch (keys) {
		case 1: this->keys = GS_KEY_1; break;
		case 2: this->keys = GS_KEY_2; break;
		case 3: this->keys = GS_KEY_3; break;
		case 4: this->keys = GS_KEY_4; break;
		default: this->keys = GS_KEY_2;
	}
	type = GO_DOOR;
}


GameState Door::frameEventQueued(WayPoint* currentWP, GameState gs) {
	// create the game behaviour of a door
	if (currentWP == trigger && gs >= keys && gs < GS_DOOR_OPEN) { 
		room->unlock();
		myNode->setVisible(false);
		return GS_DOOR_OPEN; 
	}
	return gs;
}

void Door::init(Room *room) {
	// (re)initialize the door for the game
	this->room = room;
	trigger = room->getDoorEvt();
	myNode->setPosition(0.5*(room->getDoorWP()->getPosition()+room->getDoorEvt()->getPosition())*mask);
	myNode->lookAt(trigger->getPosition()*mask, SceneNode::TS_WORLD);
	myNode->setVisible(true);
	room->lock();
	
	initialized = true;
}
