#include <Treasure.h>
using namespace Ogre;

Treasure::Treasure(SceneManager* mSceneMgr) : GameObject(mSceneMgr) {
	Entity *ent = mSceneMgr->createEntity("TreasureBox.mesh");
	ent->setMaterialName("roculus3D/Game_TreasureBox");
	myNode->attachObject(ent);
	
	gold = mSceneMgr->createParticleSystem("treasureParticlesGold","roculus3D/Game_TreasureGold");
	myNode->attachObject(gold);
	gold->setEmitting(true);
	gold->setVisible(true);
	fireworks = mSceneMgr->createParticleSystem("treasureParticlesFireworks","roculus3D/Game_TreasureFireworks");
	myNode->attachObject(fireworks);
	fireworks->setEmitting(true);
	fireworks->setVisible(true);
	
	type = GO_TREASURE;
}

GameState Treasure::frameEventQueued(WayPoint* currentWP, GameState gs) {
	if (currentWP == trigger && GS_DOOR_OPEN == gs) {
		gold->setEmitting(true);
		gold->setVisible(true);
		fireworks->setEmitting(true);
		fireworks->setVisible(true);
		return GS_FOUND_TREASURE;
	}
	return gs;
}

void Treasure::init(Room *room) {
	trigger = room->getWPs2Use()[std::rand() % room->getWPs2Use().size()];
	this->room = room;
	gold->setEmitting(false);
	gold->setVisible(false);
	fireworks->setEmitting(false);
	fireworks->setVisible(false);
	myNode->setPosition(trigger->getPosition());
	myNode->lookAt(room->getDoorWP()->getPosition(), SceneNode::TS_WORLD);
	myNode->setVisible(true);
	//~ gold->setEmitting(true);
	//~ gold->setVisible(true);
	//~ fireworks->setEmitting(true);
	//~ fireworks->setVisible(true);
	
	initialized = true;
}
