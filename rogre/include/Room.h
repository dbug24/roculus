#ifndef _ROOM_H_
#define _ROOM_H_

class Room {
  protected:
	Game *game;
	int id;
	std::vector<int> roomPoints;
	bool locked;
	Ogre::Vector3 doorPosition;
	int doorEventWPId;
	std::vector<int> usePoints;
  public:
	Room(Game*, int);
	std::vector<int>& getWPs();
	bool isLocked();
	void lock();
	void unlock();
	int getRoomId();
	std::vector<int>& getRoomPoints();
	std::vector<int>& getUsePoints();
	int getDoorEvtWPId();
	void setDoorEventWPId(int);
	void addRoomPoint(int);
	void addUsePoint(int);
	void setDoorPosition(Ogre::Vector3);
};

#endif
