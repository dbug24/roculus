#ifndef _SNAPSHOT_H_
#define _SNAPSHOT_H_

#include <OgreImage.h>
#include <OgreTexture.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>

/** \brief Collects all data that describes a Snapshot and provides a method to insert it into the scene. 
 * (Not the smartest implementation, should probably be constructed given the scene manager and do the object creation by itself.
 * Static counter would help to identify total number of Snapshots to link to individual materials, textures etc.)*/
class Snapshot
{
public:
	Snapshot(Ogre::Entity*, Ogre::SceneNode*, const Ogre::TexturePtr&, const Ogre::TexturePtr&);
	/**< Initializes this object with an Ogre::Entity (an instance of the camera geometry), a SceneNode (on which the object is placed),
	 * and two texture pointers for the depth-information (first pointer) and the rgb-information (second pointer).*/
	~Snapshot();
	/**< Default destructor.*/
	
	virtual Ogre::SceneNode* getTargetSceneNode();		/**< Returns the scene node of this object.*/
	virtual Ogre::TexturePtr getAssignedDepthTexture();	/**< Returns the depth-texture pointer for this snapshot.*/
	//~ virtual Ogre::TexturePtr getAssignedDepthMask();
	virtual Ogre::TexturePtr getAssignedRGBTexture();	/**< Returns the rgb-texture pointer for this snapshot.*/
	virtual void setTargetSceneNode(Ogre::SceneNode*);	/**< Sets the scene node for this object. (Not used currently - DEPRECATED).*/
	virtual void assignDepthTexture(const Ogre::TexturePtr&);	/**< Assign a depth-texture to the snapshot.*/
	//~ virtual void assignDepthMask(const Ogre::TexturePtr&);
	virtual void assignRGBTexture(const Ogre::TexturePtr&);	/**< Assign a rgb-texture to the snapshot.*/
	
	virtual bool placeInScene(const Ogre::Image&, const Ogre::Image&, const Ogre::Vector3&, const Ogre::Quaternion&);
	/**< Given a depth image (1st), a rgb image (2nd), the corresponding camera position and orientation, place a snapshot in the scene.*/
	
protected:	
	Ogre::Entity *snapshot;			/**< Ogre::Entity for this snapshot. (An instance of the camera geometry).*/
	Ogre::TexturePtr depthTexture;	/**< Pointer to the depth texture for this snapshot.*/
	Ogre::TexturePtr rgbTexture;	/**< Pointer to the rgb texture for this snapshot.*/
	//~ Ogre::TexturePtr depthMask;	
	Ogre::SceneNode *targetSceneNode;	/**< This snapshot's scene node.*/
	bool attached;					/**< Was this snapshot already attached to its scene node? (Needed in placeInScene(...))*/
};

#endif
