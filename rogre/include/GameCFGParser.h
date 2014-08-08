#ifndef _GAME_CFG_PARSER_H_
#define _GAME_CFG_PARSER_H_

#include <OgreConfigFile.h>
#include <OgreStringConverter.h>
#include <OgreResourceGroupManager.h>
#include <map>
#include <vector>
#include <string>
#include "WayPoint.h"

class GameCFGParser {
  protected:
	GameCFGParser();
	~GameCFGParser();
	GameCFGParser(const GameCFGParser&);
	GameCFGParser& operator=(const GameCFGParser&);
	
	Ogre::ConfigFile game_cfg;
	std::map<std::string, std::string> m_Config;
	int cntCorridors;
	int cntRooms;
	
  public:
	static GameCFGParser &getInstance();
	
	std::string getInitNode();
	int getNrKeys();
	int getNrRooms();
	int getNrCorridors();
	int getNrWayPoints();
	int getDoor(const std::string&);
	int getDoorEvt(const std::string&);
	std::vector<int> getWPs2Use(const std::string&);
	std::vector<int> getWPs(const std::string&);
	std::string getValueAsString(const std::string&);
	bool getKeyExists(const std::string&);
};

#endif
