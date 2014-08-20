#ifndef _GAME_CFG_PARSER_H_
#define _GAME_CFG_PARSER_H_

#include <OgreConfigFile.h>
#include <OgreStringConverter.h>
#include <OgreResourceGroupManager.h>
#include <map>
#include <vector>
#include <string>
#include "WayPoint.h"

/** \brief Parses the 'game.cfg' file.
 * This class provides an easy-to-use parser for the config file of a game. Note that the filename is hardcoded in the constructor.
 */
class GameCFGParser {
  protected:
	GameCFGParser();
	/**< Default constructor. Hidden due to Singleton pattern.*/
	~GameCFGParser();
	/**< Default destructor. Hidden due to Singleton pattern.*/
	GameCFGParser(const GameCFGParser&);
	/**< Copy constructor. Hidden due to Singleton pattern.*/
	GameCFGParser& operator=(const GameCFGParser&);
	/**< Assginment operator. Hidden due to Singleton pattern.*/
	
	Ogre::ConfigFile game_cfg;						/**< Stores the config file to parse.*/
	std::map<std::string, std::string> m_Config;	/**< Collects the key-value pairs.*/
	int cntCorridors;								/**< Number of Corridors.*/
	int cntRooms;									/**< Number of Rooms. */
	
  public:
	static GameCFGParser &getInstance();
	/**< Get the (single) instance of this class.*/
	
	std::string getInitNode();
	/**< Returns the node at which the game should be started (with the robot on it).*/
	int getNrKeys();
	/**< Returns the number of keys that are placed in the game.*/
	int getNrRooms();
	/**< Retruns the number of rooms in the game.*/
	int getNrCorridors();
	/**< Returns the number of corridors in the game.*/
	int getNrWayPoints();
	/**< Returns the number of waypoints in the game.*/
	int getDoor(const std::string&);
	/**< Returns the index of the 'door' waypoint in the specified room.*/
	int getDoorEvt(const std::string&);
	/**< Returns the index of the 'door event' waypoint in the specified room.*/
	std::vector<int> getWPs2Use(const std::string&);
	/**< Returns the indices of the waypoints to use for the placement of treasures or keys in the specified room.*/
	std::vector<int> getWPs(const std::string&);
	/**< Return all waypoints that belong to the specified room.*/
	std::string getValueAsString(const std::string&);
	/**< Utility method to get a value for the specified key.*/
	bool getKeyExists(const std::string&);
	/**< Checks whether the specified key exists (return true), or not (false).*/
};

#endif
