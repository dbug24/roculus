#include <Key.h>
#include <OgreEntity.h>
using namespace Ogre;

Key::Key(SceneManager* mSceneMgr) : GameObject(mSceneMgr), found(false) {
	Entity *ent = mSceneMgr->createEntity("Key.mesh");
	ent->setMaterialName("roculus3D/Game_Key");
	myNode->attachObject(ent);
}

GameState Key::frameEventQueued(WayPoint* currentWP, GameState gs) {
	if (false == found && currentWP == trigger && gs < GS_KEY_4) {
		found = true;
		myNode->setVisible(false);
		if (GS_START == gs) return GS_KEY_1;
		if (GS_KEY_1 == gs) return GS_KEY_2;
		if (GS_KEY_2 == gs) return GS_KEY_3;
		if (GS_KEY_3 == gs) return GS_KEY_4;
		return GS_KEY_4;
	}
	return gs;
}
