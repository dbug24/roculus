#include <WayPoint.h>
using namespace Ogre;

WayPoint::WayPoint(int id, SceneManager* mSceneMgr, Vector3 pos, Quaternion, WayPoint_Role role) {
	this->id = id;
	this->mSceneMgr = mSceneMgr;
	this->name = String("WayPoint")+String(boost::lexical_cast<std::string>(id));
	this->wpSN = mSceneMgr->getRootSceneNode()->createChildSceneNode(name);
	this->pos = pos;
	this->ori = ori;
	this->role = role;
	this->accessible = true;
	this->wpEnt = mSceneMgr->createEntity("Torus.001.mesh");
	wpEnt->setMaterialName("roculus3D/WayPoint");
	wpSN->setPosition(pos);
	wpSN->setOrientation(ori);
	wpSN->attachObject(wpEnt);
	wpSN->setVisible(true);
}

String WayPoint::getName() {
	return this->name;
}

const Vector3& WayPoint::getPosition() {
	return pos;
}

const Quaternion& WayPoint::getOrientation() {
	return ori;
}

int WayPoint::getId() {
	return id;
}

WayPoint_Role WayPoint::getRole() {
	return role;
}

void WayPoint::setRole(WayPoint_Role role) {
	boost::mutex::scoped_lock lock(WAYPOINT_MUTEX);
	this->role = role;
}

void WayPoint::setPosition(const Vector3& pos) {
	boost::mutex::scoped_lock lock(WAYPOINT_MUTEX);
	this->pos = pos;
	wpSN->setPosition(pos);
}

void WayPoint::setOrientation(const Quaternion& ori) {
	boost::mutex::scoped_lock lock(WAYPOINT_MUTEX);
	this->ori = ori;
	wpSN->setOrientation(ori);
}

bool WayPoint::isAccessible() {
	return accessible;
}

void WayPoint::setAccessibility(bool val) {
	boost::mutex::scoped_lock lock(WAYPOINT_MUTEX);
	this->accessible = val;
	if (true == val) {
		wpEnt->setMaterialName("roculus3D/WayPoint");
	} else {
		wpEnt->setMaterialName("roculus3D/WayPointTransparent");
	}
}

void WayPoint::setVisible(bool visible) {
	boost::mutex::scoped_lock lock(WAYPOINT_MUTEX);
	wpSN->setVisible(visible, true);
}

std::string WayPoint::toString() {
	return "WayPoint" + boost::lexical_cast<std::string>(id) + ", " + StringConverter::toString(pos) + ", " + StringConverter::toString(ori);
}

