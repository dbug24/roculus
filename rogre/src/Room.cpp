#include <Room.h>

Room::Room(int id) {
	this->id = id;
	locked = false;
	roomPoints.clear();
	usePoints.clear();
}

std::vector<int>& Room::getWPs() {
	return roomPoints;
}

bool Room::isLocked() {
	return locked;
}

void Room::lock(const std::vector<WayPoint*>& wpList) {
	for (int i=0;i<roomPoints.size();i++) {
		wpList.at(roomPoints[i])->setAccessibility(false);
	}
	locked = true;
}

void Room::unlock(const std::vector<WayPoint*>& wpList) {
	for (int i=0;i<roomPoints.size();i++) {
		wpList.at(roomPoints[i])->setAccessibility(true);
	}
	locked = false;
}

int Room::getRoomId() {
	return id;
}

std::vector<int>& Room::getRoomPoints() {
	return roomPoints;
}

std::vector<int>& Room::getUsePoints() {
	return usePoints;
}

int Room::getDoorEvt() {
	return doorEventWPId;
}

void Room::setDoorEvtId(int id) {
	this->doorEventWPId = id;
}

void Room::addRoomPoint(int id) {
	roomPoints.push_back(id);
}

void Room::addUsePoint(int id) {
	usePoints.push_back(id);
}

void Room::setDoorId(int id) {
	this->doorId = id;
}

std::string Room::toString() {
	std::string result("Room:\n");
	result.append("\t id:" + boost::lexical_cast<std::string>(id) + "\n");
	result.append("\t door:" + boost::lexical_cast<std::string>(doorId) + "\n"); 
	result.append("\t doorEvt:" + boost::lexical_cast<std::string>(doorEventWPId) + "\n"); 
	for (int i=0;i<roomPoints.size();i++) {
		result.append("\t" + boost::lexical_cast<std::string>(roomPoints[i]));
	}
	result.append("\n");
	return result;
}
