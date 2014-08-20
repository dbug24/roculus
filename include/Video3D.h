#ifndef _VIDEO3D_H_
#define _VIDEO3D_H_

#include <OgreImage.h>
#include <OgreTexture.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>

/** \brief Handles the 3D video stream.
 * Similar to a Snapshot, but uses DYNAMIC and DISCARDABLE textures instead, which are updated, not placed.
 * (Not the smartest implementation, should probably be inherit properties from Snapshot...)
 */
class Video3D
{
public:
	Video3D(Ogre::Entity*, Ogre::SceneNode*, const Ogre::TexturePtr&, const Ogre::TexturePtr&);
	/**< See constructor of Snapshot class.*/
	~Video3D();
	/**< Default destructor.*/
	
	virtual Ogre::SceneNode* getTargetSceneNode();		/**< The scene node of the video stream.*/
	virtual Ogre::TexturePtr getAssignedDepthTexture();	/**< Get tointer to the depth texture of the 3D stream.*/
	//~ virtual Ogre::TexturePtr getAssignedDepthMask();
	virtual Ogre::TexturePtr getAssignedRGBTexture();	/**< Get the pointer to the rgb texture of the stream.*/
	virtual void setTargetSceneNode(Ogre::SceneNode*);	/**< Set the scene node. (Not used, done during construction.*/
	virtual void assignDepthTexture(const Ogre::TexturePtr&);	/**< Assign a depth texture.*/
	//~ virtual void assignDepthMask(const Ogre::TexturePtr&);
	virtual void assignRGBTexture(const Ogre::TexturePtr&);	/**< Assign a rgb texture.*/
	
	virtual bool update(const Ogre::Image&, const Ogre::Image&, const Ogre::Vector3&, const Ogre::Quaternion&);
	/**< Update the video stream with the new: (1) depth image, (2) rgb image, (3) position and (4) orientation.*/
	
protected:	
	Ogre::Entity *snapshot;				/**< The entity of the video.*/
	Ogre::TexturePtr depthTexture;		/**< Pointer to the depth texture.*/
	Ogre::TexturePtr rgbTexture;		/**< Pointer to the rgb texture.*/
	//~ Ogre::TexturePtr depthMask;
	Ogre::SceneNode *targetSceneNode;	/**< The scene node of the video.*/
	bool attached;						/**< Was this object already attached to its scene node?*/
};

#endif
