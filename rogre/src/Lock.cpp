#include <Lock.h>

Lock::Lock(SceneManager* mSceneMgr, int idx) : GameObject(mSceneMgr), locked(true) {
	this->idx = idx;
	
	ent = mSceneMgr->createEntity("Lock.mesh");
	ent->setMaterialName("roculus3D/Game_Lock");
	myNode->attachObject(ent);
	myNode->setScale(Vector3(0.5f, 0.5f,0.5f));
	
	if (0 == idx) { react = GS_KEY_1; }
	else if (1 == idx) { react = GS_KEY_2; }
	else if (2 == idx) { react = GS_KEY_3; }
	else if (3 == idx) { react = GS_KEY_4; }
	else { react = GS_START; }
	
	type = GO_LOCK;
}

GameState Lock::frameEventQueued(WayPoint* currentWP, GameState gs) {
	myNode->yaw(-Degree(0.5));
	if (gs == react && true == locked) {
		locked = false;
		ent->setMaterialName("roculus3D/Game_Lock_Open");
	}
	if (currentWP == trigger && gs >= react) {
		myNode->setVisible(false);
	}
	return gs;
}

void Lock::init(Room *room) {
	trigger = room->getDoorEvt();
	ent->setMaterialName("roculus3D/Game_Lock");
	myNode->setPosition(trigger->getPosition() + Vector3(0.0f, 0.4f*float(idx+0.5), 0.0f));
	myNode->setOrientation(Quaternion::IDENTITY);
	myNode->setVisible(true);
	locked = true;
	
	initialized = true;
}
