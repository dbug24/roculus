#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "Snapshot.h"

class SnapshotLibrary {
public:
	void allocate(int);
	bool placeInScene(const Ogre::Image&, const Ogre::Image&, const Ogre::Vector3& , const Ogre::Quaternion&);
	SnapshotLibrary(Ogre::SceneManager*, const Ogre::String&, const Ogre::String&, int);
	~SnapshotLibrary();
protected:
	std::vector<Snapshot*> library;
	int currentSnapshot;
	int maxSnapshots;
	Ogre::String EntityPrototype;
	Ogre::String MaterialPrototype;
	Ogre::SceneManager *mSceneMgr;
};
