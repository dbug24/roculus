#include <OgreImage.h>
#include <OgreTexture.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>

class Video3D
{
public:
	Video3D(Ogre::Entity*, Ogre::SceneNode*, const Ogre::TexturePtr&, const Ogre::TexturePtr&);
	~Video3D();
	
	virtual Ogre::SceneNode* getTargetSceneNode();
	virtual Ogre::TexturePtr getAssignedDepthTexture();
	//~ virtual Ogre::TexturePtr getAssignedDepthMask();
	virtual Ogre::TexturePtr getAssignedRGBTexture();
	virtual void setTargetSceneNode(Ogre::SceneNode*);
	virtual void assignDepthTexture(const Ogre::TexturePtr&);
	//~ virtual void assignDepthMask(const Ogre::TexturePtr&);
	virtual void assignRGBTexture(const Ogre::TexturePtr&);
	
	virtual bool update(const Ogre::Image&, const Ogre::Image&, const Ogre::Vector3&, const Ogre::Quaternion&);
	
protected:	
	Ogre::Entity *snapshot;
	Ogre::TexturePtr depthTexture;
	Ogre::TexturePtr rgbTexture;
	//~ Ogre::TexturePtr depthMask;
	Ogre::SceneNode *targetSceneNode;
	bool attached;
};
