#include <Treasure.h>
using namespace Ogre;

Treasure::Treasure(SceneManager* mSceneMgr) : GameObject(mSceneMgr) {
	Entity *ent = mSceneMgr->createEntity("TreasureBox.mesh");
	ent->setMaterialName("roculus3D/Game_TreasureBox");
	myNode->attachObject(ent);
}

GameState Treasure::frameEventQueued(WayPoint* currentWP, GameState gs) {
	if (currentWP == trigger && GS_DOOR_OPEN == gs) return GS_FOUND_TREASURE;
	return gs;
}

void Treasure::init(Room *room) {
	trigger = room->getWPs2Use()[std::rand() % room->getWPs2Use().size()];
	this->room = room;
	myNode->setPosition(trigger->getPosition());
	myNode->lookAt(room->getDoorWP()->getPosition(), SceneNode::TS_WORLD);
	myNode->setVisible(true);
	
	initialized = true;
}
