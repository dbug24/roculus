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

/**< \brief Groups multiple Snapshots in a library (vector). Furthermore, this class manages the memory and preallocates
 * Ogre::Textures, Materials and SceneNodes, whenever needed.
 */
class SnapshotLibrary {
public:
	bool placeInScene(const Ogre::Image&, const Ogre::Image&, const Ogre::Vector3& , const Ogre::Quaternion&);
	/**< Places a new snapshot in the scene. Basically, this is done by forwarding the command to the Snapshot class, but it involves some memory check beforehand.*/
	void flipVisibility();
	/**< Toggle the visiblity of all snapshots in the library.*/
    SnapshotLibrary(Ogre::SceneManager*, const Ogre::String&, const Ogre::String&, int);
    /**< Initialize the object with: (1) the scene manager (for object creation), (2) the entity prototype for the camera geometry, (3) the default material and
     * (4) the number of snapshots for which memory should be preallocated each time.*/
	~SnapshotLibrary();
	/**< Default destructor.*/
protected:
	void allocate(int);
	/**< Utility method to allocate new memory for a number of snapshots.*/
	std::vector<Snapshot*> library;		/**< The vector of Snapshot objects.*/
	int currentSnapshot;				/**< The current snapshot index.*/
	int maxSnapshots;					/**< The current maximal size of the library.*/
	Ogre::String EntityPrototype;		/**< The entity prototype.*/
	Ogre::String MaterialPrototype;		/**< The material prototype.*/
	Ogre::SceneManager *mSceneMgr;		/**< The scene manager.*/
	Ogre::SceneNode *mMasterSceneNode;	/**< The scene node of this library.*/
};

#endif
