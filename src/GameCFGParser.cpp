#include <GameCFGParser.h>
using namespace Ogre;

GameCFGParser::GameCFGParser() {
	game_cfg.load(String("game.cfg"), "=", true);
	
	Ogre::ConfigFile::SectionIterator seci = game_cfg.getSectionIterator();
	Ogre::String sectionName;
	Ogre::String keyName;
	Ogre::String valueName;
 
	while (seci.hasMoreElements())
	{
		sectionName = seci.peekNextKey();
		
		if (sectionName.compare("Corridor") == 0) {
			cntCorridors++;
		} else if (sectionName.compare("Room") == 0) {
			cntRooms++;
		}
		
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator i;
		for (i = settings->begin(); i != settings->end(); ++i)
		{
			keyName = i->first;
			valueName = i->second;
			if (sectionName.compare("Game") == 0) {
				m_Config.insert(std::pair<std::string, std::string>(sectionName + "/" + keyName, valueName));
			} else if (sectionName.compare("Corridor") == 0)  {
				m_Config.insert(std::pair<std::string, std::string>(sectionName + "/" + keyName, valueName));
			} else {
				m_Config.insert(std::pair<std::string, std::string>(sectionName + "/" + keyName, valueName));
			}
		}
	}
	
}

GameCFGParser& GameCFGParser::getInstance() {
	static GameCFGParser instance;
	return instance;
}

GameCFGParser::~GameCFGParser() { }

std::string GameCFGParser::getInitNode() {
	return "WayPoint" + getValueAsString("Game/initWP");
}

int GameCFGParser::getNrKeys() {
	return StringConverter::parseInt(getValueAsString("Game/cntKeys"));
}
	
int GameCFGParser::getNrRooms() {
	return StringConverter::parseInt(getValueAsString("Game/cntRooms"));
}

int GameCFGParser::getNrCorridors() {
	return StringConverter::parseInt(getValueAsString("Game/cntCorridors"));
}

int GameCFGParser::getNrWayPoints() {
	return StringConverter::parseInt(getValueAsString("Game/cntWayPoints"));
}

int GameCFGParser::getDoor(const std::string& room) {
	return StringConverter::parseInt(getValueAsString(room + "/Door"));
}

int GameCFGParser::getDoorEvt(const std::string& room) {
	return StringConverter::parseInt(getValueAsString(room + "/DoorEvt"));
}

std::vector<int> GameCFGParser::getWPs2Use(const std::string& room) {
	std::vector<int> result;
	StringVector v_str = StringConverter::parseStringVector(getValueAsString(room + "/WPs2Use"));
	for (int i=0;i<v_str.size();i++) {
		result.push_back(StringConverter::parseInt(v_str[i]));
	}
	return result;
}

std::vector<int> GameCFGParser::getWPs(const std::string& room) {
	std::vector<int> result;
	StringVector v_str = StringConverter::parseStringVector(getValueAsString(room + "/WPs"));
	for (int i=0;i<v_str.size();i++) {
		result.push_back(StringConverter::parseInt(v_str[i]));
	}
	return result;
}

std::string GameCFGParser::getValueAsString(const std::string &key)
{
	if (getKeyExists(key) == true)
	{
		return m_Config[key];
	}
	else
	{
		throw Ogre::Exception(Ogre::Exception::ERR_ITEM_NOT_FOUND,"Configuration key: " + key + " not found", "MyConfig::getValue");
	}
 
}

bool GameCFGParser::getKeyExists(const std::string &key)
{
	if (m_Config.count(key) > 0)
	{
		return true;
	}
	return false;
}
