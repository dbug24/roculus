#ifndef _GLOBAL_MAP_H_
#define _GLOBAL_MAP_H_

#include <OgreSceneManager.h>
#include <OgreTextureManager.h>
#include <OgreManualObject.h>
#include <OgreEntity.h>

using namespace Ogre;

class GlobalMap {
public:
	GlobalMap(SceneManager*);
	~GlobalMap();
	void insertWHR(uint32, uint32, Real);
	void setOrigin(Vector3);
	void includeMap(const Image&);
	void flipVisibility();
	Real getWidth();
	Real getHeight();
	Real getResolution();
	Vector3 getOrigin();
protected:
	Real width;
	Real height;
	Real resolution;
	Vector3 origin;
	SceneManager *mSceneMgr;
	ManualObject *mMapObj;
	SceneNode *pSceneNode;
};

#endif
