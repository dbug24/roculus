#ifndef _SNAPSHOT_LIBRARY_H_
#define _SNAPSHOT_LIBRARY_H_

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include "Snapshot.h"

#include <iostream>
#include <stdio.h>
#include <exception>
#include <math.h>

#include <fstream>

class SnapshotLibrary {
public:
	void allocate(int);
	bool placeInScene(const Ogre::Image&, const Ogre::Image&, const Ogre::Vector3& , const Ogre::Quaternion&);
	void flipVisibility();
    void setSaveOnShutdown(bool);
    SnapshotLibrary(Ogre::SceneManager*, const Ogre::String&, const Ogre::String&, int, bool saveOnShutdown=false);
	~SnapshotLibrary();
protected:
	//~ void saveMap();
	std::vector<Snapshot*> library;
	int currentSnapshot;
	int maxSnapshots;
	Ogre::String EntityPrototype;
	Ogre::String MaterialPrototype;
	Ogre::SceneManager *mSceneMgr;
	Ogre::SceneNode *mMasterSceneNode;
	bool save;
//    std::vector<const Ogre::Image&> m_DepthBuffer;
};

#endif
