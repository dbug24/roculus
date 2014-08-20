#ifndef _ROOM_H_
#define _ROOM_H_

#include <string.h>
#include <WayPoint.h>

/**< \brief Describes a Room in a Game.
 * This class collects all parameters for a Room. It keeps track of the corresponding WayPoints, Door nodes and event nodes and provides some utility functions.
 */
class Room {
  protected:
	int id;								/**< The number/ID of this room.*/
	std::vector<WayPoint*> roomPoints;	/**< All WayPoints that belong to this room.*/
	bool locked;						/**< Stores if the room is locked.*/
	WayPoint* door;						/**< In case this room will be used as Treasure chamber, which WayPoint marks the Door?*/
	WayPoint* doorEventWP;				/**< In case this room will be used as Treasure chamber, which WayPoint triggers the Door event?*/
	std::vector<WayPoint*> usePoints;	/**< Which WayPoints can be used to place Keys or the Treasure?*/
  public:
	Room(int);							/**< Initialize the room, assigning its ID.*/
	std::vector<WayPoint*>& getWPs();	/**< Returns the vector of all WayPoints that are in this room.*/
	std::vector<WayPoint*>& getWPs2Use();	/**< Returns the vector of WayPoints that can be used to place Keys/Treasures.*/
	bool isLocked();					/**< Is this room locked?*/
	void lock();						/**< Lock this room: make its WayPoints unavailable for selection.*/
	void unlock();						/**< Unlock the room: make its WayPoints accessible again.*/
	int getRoomId();					/**< Returns the ID of this room.*/
	WayPoint* getDoorEvt();				/**< Returns the WayPoint which triggers the door event.*/
	WayPoint* getDoorWP();				/**< Returns the WayPoint which denotes the door. In fact the door will be placed half way between the doorWP and the doorEventWP.*/
	void setDoorEvt(WayPoint*);			/**< Set the WayPoint which triggers the door event.*/
	void addRoomPoint(WayPoint*);		/**< Add a WayPoint to the room.*/
	void addUsePoint(WayPoint*);		/**< Add a WayPoint to the set of WayPoints to use for Keys/Treasures.*/
	void setDoor(WayPoint*);			/**< Set the WayPoint which denotes the door.*/
	void print();						/**< Utility function. Print the room configuration on the console. Useful for debugging.*/
};

#endif
