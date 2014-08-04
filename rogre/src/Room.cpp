#include <Room.h>

using namespace Room;

Room(Game *game, int id) {
	this->id = id;
	this->game = game;
	locked = false;
}

std::vector<int>& getWPs() {
	return roomPoints;
}

bool isLocked() {
	return locked;
}

void lock() {
	for (int i=0;i<roomPoints.size();i++) {
		game->getWPs()->at(roomPoints[i])->setAccessibility(false);
	}
	locked = true;
}

void unlock() {
	for (int i=0;i<roomPoints.size();i++) {
		game->getWPs()->at(roomPoints[i])->setAccessibility(true);
	}
	locked = false;
}

int getRoomId() {
	return id;
}

std::vector<int>& getRoomPoints() {
	return roomPoints;
}

std::vector<int>& getUsePoints() {
	return usePoints;
}

int getDoorEventWPId() {
	return doorEventWPId;
}

void setDoorEventWPId(int id) {
	this->doorEventWPId = id;
}

void addRoomPoint(int id) {
	roomPoints.push_back(id);
}

void addUsePoint(int id) {
	usePoints.push_back(id);
}

void setDoorPosition(Ogre::Vector3 pos) {
	this->doorPosition = pos;
}
