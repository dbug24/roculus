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
