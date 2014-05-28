#ifndef _SNAPSHOT_LIBRARY_H_
#define _SNAPSHOT_LIBRARY_H_

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include "Snapshot.h"

class SnapshotLibrary {
public:
	void allocate(int);
	bool placeInScene(const Ogre::Image&, const Ogre::Image&, const Ogre::Vector3& , const Ogre::Quaternion&);
	void flipVisibility();
    void saveMap();
    SnapshotLibrary(Ogre::SceneManager*, const Ogre::String&, const Ogre::String&, int, bool bufferSnapshotData=false);
	~SnapshotLibrary();
protected:
	std::vector<Snapshot*> library;
	int currentSnapshot;
	int maxSnapshots;
	Ogre::String EntityPrototype;
	Ogre::String MaterialPrototype;
	Ogre::SceneManager *mSceneMgr;
	Ogre::SceneNode *mMasterSceneNode;
    bool        m_bBufferSnapshotData;
//    std::vector<const Ogre::Image&> m_DepthBuffer;
};

#endif
