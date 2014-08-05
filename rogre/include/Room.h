#ifndef _ROOM_H_
#define _ROOM_H_

#include <string.h>
#include <WayPoint.h>

class Room {
  protected:
	int id;
	std::vector<int> roomPoints;
	bool locked;
	int doorId;
	int doorEventWPId;
	std::vector<int> usePoints;
  public:
	Room(int);
	std::vector<int>& getWPs();
	bool isLocked();
	void lock(const std::vector<WayPoint*>& wpList);
	void unlock(const std::vector<WayPoint*>& wpList);
	int getRoomId();
	std::vector<int>& getRoomPoints();
	std::vector<int>& getUsePoints();
	int getDoorEvt();
	void setDoorEvtId(int);
	void addRoomPoint(int);
	void addUsePoint(int);
	void setDoorId(int);
	std::string toString();
};

#endif
