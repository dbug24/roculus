#include <WayPoint.h>
using namespace Ogre;

WayPoint::WayPoint(int id, SceneNode* wpSceneNode = NULL, Vector3 pos = Vector3::ZERO, Quaternion ori = Quaternion::IDENTITY, WayPoint_Role role = WP_ROLE_NONE) {
	this->id = id;
	this->wpSN = wpSceneNode;
	this->pos = pos;
	this->ori = ori;
	this->role = role;
	this->accessible = true;
	this->name = String("WayPoint")+String(boost::lexical_cast<std::string>(id));
}

String WayPoint::getName() {
	return this->name;
}

Vector3* WayPoint::getPosition() {
	return &pos;
}

Quaternion* WayPoint::getOrientation() {
	return &ori;
}

int WayPoint::getId() {
	return id;
}

WayPoint_Role WayPoint::getRole() {
	return role;
}

void WayPoint::setRole(WayPoint_Role role) {
	this->role = role;
}

void WayPoint::setPosition(const Vector3& pos) {
	this->pos = pos;
	wpSN->setPosition(pos);
}

void WayPoint::setOrientation(const Quaternion& ori) {
	this->ori = ori;
	wpSN->setOrientation(ori);
}

bool WayPoint::isAccessible() {
	return accessible;
}

void WayPoint::setAccessibility(bool val) {
	this->accessible = val;
}

SceneNode* WayPoint::getSceneNode() {
	return wpSN;
}

void WayPoint::setVisible(bool visible) {
	wpSN->setVisible(visible, true);
}

std::string WayPoint::toString() {
	return "WayPoint" + boost::lexical_cast<std::string>(id) + ", " + StringConverter::toString(pos) + ", " + StringConverter::toString(ori);
}

