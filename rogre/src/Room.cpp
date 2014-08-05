#include <Room.h>

Room::Room(int id) {
	this->id = id;
	locked = false;
	roomPoints.clear();
	usePoints.clear();
}

std::vector<WayPoint*>& Room::getWPs() {
	return roomPoints;
}

bool Room::isLocked() {
	return locked;
}

void Room::lock() {
	for (int i=0;i<roomPoints.size();i++) {
		roomPoints[i]->setAccessibility(false);
	}
	locked = true;
}

void Room::unlock() {
	for (int i=0;i<roomPoints.size();i++) {
		roomPoints[i]->setAccessibility(true);
	}
	locked = false;
}

int Room::getRoomId() {
	return id;
}

std::vector<WayPoint*>& Room::getWPs2Use() {
	return usePoints;
}

WayPoint* Room::getDoorEvt() {
	return doorEventWP;
}

void Room::setDoorEvt(WayPoint* wp) {
	this->doorEventWP = wp;
}

void Room::addRoomPoint(WayPoint* wp) {
	roomPoints.push_back(wp);
}

void Room::addUsePoint(WayPoint* wp) {
	usePoints.push_back(wp);
}

void Room::setDoor(WayPoint* wp) {
	this->door = wp;
}

void Room::print() {
	std::cout << "Room" << id << std::endl;
	if (door) std::cout << "\t door: " << door->toString() << std::endl; 
	if (doorEventWP) std::cout << "\t doorEvt: " + doorEventWP->toString() << std::endl << "\t" << "list of points:" << std::endl;
	for (int i=0;i<roomPoints.size();i++) {
		std::cout << "\t\t" << roomPoints[i]->toString() << std::endl;
	}
	std::cout << std::endl;
}
