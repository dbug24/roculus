#ifndef _GLOBAL_MAP_H_
#define _GLOBAL_MAP_H_

#include <OgreSceneManager.h>
#include <OgreTextureManager.h>
#include <OgreManualObject.h>
#include <OgreEntity.h>

using namespace Ogre;

/**< \brief Utility class for the 2D map.
 * This class handles the 2D map in the scene.
 */
class GlobalMap {
public:
	GlobalMap(SceneManager*);
	/**< Use this constructor to initialize the object!*/
	~GlobalMap();
	/**< Default destructor.*/
	void insertWHR(uint32, uint32, Real);
	/**< Set the width, height(depth), and resolution of the map-image.*/
	void setOrigin(Vector3);
	/**< Set the origin of the map.*/
	void includeMap(const Image&);
	/**< Given the image, create the map plane and texture it with the image. Place the scene node in the world.*/
	void flipVisibility();
	/**< Trigger the visibility of the map.*/
	Real getWidth();		/**< Return the width of the map.*/
	Real getHeight();		/**< Return the height(depth) of the map.*/
	Real getResolution();	/**< Return the resolution of the map.*/
	Vector3 getOrigin();	/**< Return the origin of the map.*/
protected:
	Real width;				/**< The width of the map.*/
	Real height;			/**< The height(depth) of the map.*/
	Real resolution;		/**< The reolution of the map.*/
	Vector3 origin;			/**< The origin of the map.*/
	SceneManager *mSceneMgr;/**< The scene manager.*/
	ManualObject *mMapObj;	/**< The manual object used to create the plane.*/
	SceneNode *pSceneNode;	/**< The scene node that holds the map.*/
};

#endif
