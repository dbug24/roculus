#include <GameCFGParser.h>
using namespace Ogre;

GameCFGParser::GameCFGParser(const char* filename) {
	std::string std_line;
	String line;
	std::ifstream file(filename);
	if (file.is_open()) {
		while (getline(file, std_line)) {
			//parse
			line.clear();
			line.append(std_line);
			StringUtil::trim(line, true, false);
			if (line.length() > 0 && line[0] == '#') continue; //skip comment lines
			
			StringVector parseLine(StringConverter::parseStringVector(line));
			// eventually add room definition here:
			if (parseLine.size() > 2) areas.push_back(parseLine[0]);
			if (parseLine[1].compare(":") != 0) { std::cerr << "Error parsing GameCFG file."; break; }
			int i = 2;
			while (parseLine[i].compare(":") != 0) {
				wpsNotToUse.push_back(StringConverter::parseInt(parseLine[i]));
				i++;
			}
			i++;
			while (i < parseLine.size()) {
				wpsToUse.push_back(StringConverter::parseInt(parseLine[i]));
				i++;
			}
		}
		file.close();
	}
}

void GameCFGParser::assignRoles(std::vector<WayPoint*>* wps) {
	for (int i=0; i<wpsToUse.size();i++) {
		wps->at(wpsToUse[i])->setRole(WP_ROLE_BOTH);
	}
}

std::vector<int>& GameCFGParser::getWPsToUse() {
	return wpsToUse;
}

std::vector<int>& GameCFGParser::getWPsNotToUse() {
	return wpsNotToUse;
}

StringVector& GameCFGParser::getAreas() {
	return areas;
}
