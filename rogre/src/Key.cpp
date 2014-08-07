#include <Key.h>
#include <OgreQuaternion.h>
using namespace Ogre;

Key::Key(SceneManager* mSceneMgr) : GameObject(mSceneMgr), found(false) {
	Entity *ent = mSceneMgr->createEntity("Key.mesh");
	ent->setMaterialName("roculus3D/Game_Key");
	myNode->attachObject(ent);
}

GameState Key::frameEventQueued(WayPoint* currentWP, GameState gs) {
	myNode->yaw(Degree(1));
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

void Key::init(Room *room) {	
	trigger = room->getWPs2Use()[std::rand() % room->getWPs2Use().size()];
	this->room = room;
	found = false;
	myNode->setOrientation(Quaternion::IDENTITY);
	myNode->setPosition(trigger->getPosition());
	myNode->setVisible(true);
	
	initialized = true;
}
