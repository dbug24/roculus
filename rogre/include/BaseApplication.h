/*
-----------------------------------------------------------------------------
Filename:    BaseApplication.h
-----------------------------------------------------------------------------
 
This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
	  Tutorial Framework
	  http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#ifndef __BaseApplication_h_
#define __BaseApplication_h_
 
#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
 
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
#  include <OIS/OISEvents.h>
#  include <OIS/OISInputManager.h>
#  include <OIS/OISKeyboard.h>
#  include <OIS/OISMouse.h>
 
#  include <OGRE/SdkTrays.h>
#  include <OGRE/SdkCameraMan.h>
#else
#  include <OISEvents.h>
#  include <OISInputManager.h>
#  include <OISKeyboard.h>
#  include <OISMouse.h>
 
#  include <SdkTrays.h>
#  include <SdkCameraMan.h>
#endif
 
#ifdef OGRE_STATIC_LIB
#  define OGRE_STATIC_GL
#  if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#    define OGRE_STATIC_Direct3D9
// dx10 will only work on vista, so be careful about statically linking
#    if OGRE_USE_D3D10
#      define OGRE_STATIC_Direct3D10
#    endif
#  endif
#  define OGRE_STATIC_BSPSceneManager
#  define OGRE_STATIC_ParticleFX
#  define OGRE_STATIC_CgProgramManager
#  ifdef OGRE_USE_PCZ
#    define OGRE_STATIC_PCZSceneManager
#    define OGRE_STATIC_OctreeZone
#  else
#    define OGRE_STATIC_OctreeSceneManager
#  endif
#  include "OgreStaticPluginLoader.h"
#endif
 
#include "OgreOculus.h"
#include "PlayerBody.h"
#include "Robot.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>			// Input handling
#include <sensor_msgs/CompressedImage.h>// Image and Video streams
#include <nav_msgs/OccupancyGrid.h>		// GlobalMap
#include <message_filters/subscriber.h>	// Sychronized Message Handling
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>		// Transforms
#include <scitos_ptu/PanTiltAction.h>	// Headers for the PTU panorama scan
#include <scitos_ptu/PanTiltGoal.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/thread/thread.hpp>


#include "SnapshotLibrary.h"
#include "Video3D.h"
#include "GlobalMap.h"

//~ typedef boost::mutex Lock;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> ApproximateTimePolicy;
typedef actionlib::SimpleActionClient<scitos_ptu::PanTiltAction> Client;

class BaseApplication : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener, public OIS::JoyStickListener, OgreBites::SdkTrayListener
{
public:
	BaseApplication(void);
	virtual ~BaseApplication(void);
 
	virtual void go(void);
 
protected:
	virtual bool setup();
	virtual bool configure(void);
	virtual void chooseSceneManager(void);
	virtual void createCamera(void);
	virtual void createFrameListener(void);
	virtual void createScene(void) = 0; // Override me!
	virtual void destroyScene(void);
	virtual void createViewports(void);
	virtual void setupResources(void);
	virtual void createResourceListener(void);
	virtual void loadResources(void);
	virtual bool frameStarted(const Ogre::FrameEvent& evt);
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual bool frameEnded(const Ogre::FrameEvent& evt);
	
	//Keyboard Listener:
	virtual bool keyPressed( const OIS::KeyEvent &arg );
	virtual bool keyReleased( const OIS::KeyEvent &arg );
	//Mouse Listener:
	virtual bool mouseMoved( const OIS::MouseEvent &arg );
	virtual bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
	virtual bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
	//Joystick Listener: (Designed for Gamepads):
	virtual bool povMoved( const OIS::JoyStickEvent &arg, int pov );
	virtual bool axisMoved( const OIS::JoyStickEvent &arg, int axis );
	virtual bool sliderMoved( const OIS::JoyStickEvent &arg, int sliderID );
	virtual bool buttonPressed( const OIS::JoyStickEvent &arg, int button );
	virtual bool buttonReleased( const OIS::JoyStickEvent &arg, int button );

	//ROS system and callbacks
	virtual void initROS();
	virtual void destroyROS();
	virtual void triggerPanoramaPTUScan();
	virtual void joyCallback(const sensor_msgs::Joy::ConstPtr& );
	virtual void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& );
	virtual void syncCallback(const sensor_msgs::CompressedImageConstPtr&, const sensor_msgs::CompressedImageConstPtr&);
	virtual void syncVideoCallback(const sensor_msgs::CompressedImageConstPtr&, const sensor_msgs::CompressedImageConstPtr&);
 
	//Adjust mouse clipping area
	virtual void windowResized(Ogre::RenderWindow* rw);
	//Unattach OIS before window shutdown (very important under Linux)
	virtual void windowClosed(Ogre::RenderWindow* rw);
 
	Ogre::Root *mRoot;
	Ogre::Camera* mCamera;
	Ogre::SceneManager* mSceneMgr;
	Ogre::RenderWindow* mWindow;
	Ogre::String mResourcesCfg;
	Ogre::String mPluginsCfg;

	Ogre::OverlaySystem *mOverlaySystem;
 
	// OgreBites
	OgreBites::InputContext mInputContext;
	OgreBites::SdkTrayManager* mTrayMgr;
	OgreBites::SdkCameraMan* mCameraMan;     	// basic camera controller
	OgreBites::ParamsPanel* mDetailsPanel;   	// sample details panel
	bool mCursorWasVisible;						// was cursor visible before dialog appeared
	bool mShutDown;
 
	//OIS Input devices
	OIS::InputManager* mInputManager;
	OIS::Mouse*    mMouse;
	OIS::Keyboard* mKeyboard;
	OIS::JoyStick* mJoyStick;
	PlayerBody* mPlayer;
	Ogre::SceneNode *mPlayerBodyNode;

	//Oculus Rift Support (Kojack)
	Oculus* oculus;

	//ROS connection
	Client *rosPTUClient;
	boost::thread *ptuSweep;
	ros::AsyncSpinner* hRosSpinner;
	ros::NodeHandle* hRosNode;
	ros::Subscriber *hRosSubJoy, *hRosSubMap;
	message_filters::Subscriber<sensor_msgs::CompressedImage> *hRosSubRGB, *hRosSubDepth, *hRosSubRGBVid, *hRosSubDepthVid;
	message_filters::Synchronizer<ApproximateTimePolicy> *rosMsgSync, *rosVideoSync;
	tf::TransformListener *tfListener;
	tf::StampedTransform snTransform, vdTransform;
	Robot *robotModel;
	GlobalMap *globalMap;

	Ogre::ManualObject *mPCRender;
	Ogre::Image depImage, texImage, depVideo, texVideo, mapImage;
	//~ Lock lockRendering;
	SnapshotLibrary *snLib, *rsLib;
	Video3D *vdVideo;
	Ogre::Vector3 snPos, vdPos;
	Ogre::Quaternion snOri, vdOri;
	bool syncedUpdate, videoUpdate, takeSnapshot, mapArrived;
	
	// Added for Mac compatibility
	Ogre::String                 m_ResourcePath;
 
#ifdef OGRE_STATIC_LIB
	Ogre::StaticPluginLoader m_StaticPluginLoader;
#endif
};
 
#endif // #ifndef __BaseApplication_h_
