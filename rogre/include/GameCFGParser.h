#ifndef _GAME_CFG_PARSER_H_
#define _GAME_CFG_PARSER_H_

#include <OgreString.h>
#include <OgreStringVector.h>
#include <OgreStringConverter.h>
#include <fstream>
#include <vector>
#include <string>
#include "WayPoint.h"

class GameCFGParser {
  protected:
	std::vector<int> wpsToUse;
	std::vector<int> wpsNotToUse;
	Ogre::StringVector areas;	
  public:
	GameCFGParser(const char*);
	std::vector<int>& getWPsToUse();
	std::vector<int>& getWPsNotToUse();
	Ogre::StringVector& getAreas();
	void assignRoles(std::vector<WayPoint*>*);
};

#endif
