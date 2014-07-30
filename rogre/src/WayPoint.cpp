#include <WayPoint.h>
using namespace Ogre;

WayPoint::WayPoint(int id) {
	this->id = id;
	this->accessible = true;
	this->name = String("WayPoint")+String(boost::lexical_cast<std::string>(id));
}

WayPoint::WayPoint(int id, Vector3 pos, WayPoint_Role role) {
	this->id = id;
	this->pos = pos;
	this->role = role;
	this->accessible = true;
	this->name = String("WayPoint")+String(boost::lexical_cast<std::string>(id));
}

String WayPoint::getName() {
	return this->name;
}

Vector3 WayPoint::getPosition() {
	return this->pos;
}

int WayPoint::getId() {
	return this->id;
}

WayPoint_Role WayPoint::getRole() {
	return this->role;
}

void WayPoint::setRole(WayPoint_Role role) {
	this->role = role;
}

void WayPoint::setPosition(Vector3 pos) {
	this->pos = pos;
}

bool WayPoint::isAccessible() {
	return accessible;
}

void WayPoint::setAccessibility(bool val) {
	this->accessible = val;
}

