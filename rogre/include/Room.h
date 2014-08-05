#ifndef _ROOM_H_
#define _ROOM_H_

#include <string.h>
#include <WayPoint.h>

class Room {
  protected:
	int id;
	std::vector<WayPoint*> roomPoints;
	bool locked;
	WayPoint* door;
	WayPoint* doorEventWP;
	std::vector<WayPoint*> usePoints;
  public:
	Room(int);
	std::vector<WayPoint*>& getWPs();
	std::vector<WayPoint*>& getWPs2Use();
	bool isLocked();
	void lock();
	void unlock();
	int getRoomId();
	WayPoint* getDoorEvt();
	void setDoorEvt(WayPoint*);
	void addRoomPoint(WayPoint*);
	void addUsePoint(WayPoint*);
	void setDoor(WayPoint*);
	void print();
};

#endif
