#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <tf/transform_listener.h>

class Robot {
public:
	Robot(Ogre::SceneManager*);
	~Robot();
	
	virtual void updateFrom(tf::TransformListener*);
	virtual Ogre::SceneNode* getSceneNode();
protected:
	Ogre::SceneNode *robot;
};
