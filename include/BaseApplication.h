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

// for the game:
#include <compressed_depth_image_transport/compression_common.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <topological_navigation/GotoNodeAction.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "SnapshotLibrary.h"
#include "Video3D.h"
#include "GlobalMap.h"
#include "WayPoint.h"
#include "Game.h"

/** typedef for the synchronized message handling (ROS) */
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> ApproximateTimePolicy;

/** typedef for the Pan-Tilt-Unit control on the robot */
typedef actionlib::SimpleActionClient<scitos_ptu::PanTiltAction> Client;

/** \brief Ogre BaseApplication class
 * 	The BaseApplication class is the central class in the project. It brings all systems together:
 * 	*initialization and control of the rendering process
 *  *handling keyboard/mouse input
 *  *connection to ROS for joystick input, image sensor data, waypoint data, etc.
 *  *cleaning up after the application finished
 **/
class BaseApplication : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener, OgreBites::SdkTrayListener
{
public:
	BaseApplication(void);
	/**< Default constructor. Initialize all members to default values. */
	
	virtual ~BaseApplication(void);
	/**< Default destructor. Final cleanup of elements. */
 
	virtual void go(void);
	/**< Start the application (reimplemented in Roculus.cpp). */
 
protected:
	virtual bool setup();
	/**< Take all necessary steps to setup the application. Chose the SceneMgr., create resource listenener, create the scene and initiate Game and ROS */
	
	virtual bool configure(void);
	/**< Parse the file ogre.cfg, or show the config dialog if the file is corrupt. */
	
	virtual void chooseSceneManager(void);
	/**< Set up a GENERIC scene manager and initiate the Overlay System. */
	virtual void createFrameListener(void);
	/**< Sets up the FrameListener, which will register the input components handled by OIS (Object Oriented Input System) and configure the info-panels */
	virtual void createScene(void) = 0;
	/**< Create the displayed environment. This method has to be reimplemented in the Roculus class. */
	virtual void destroyScene(void);
	/**< Clean-up the displayed environment (if necessary). Currenty does nothing. */
	virtual void setupResources(void);
	/**< Parses the resource.cfg to load the specified files (scripts, textures, shaders, ...) into memory. */
	virtual void createResourceListener(void);
	/**< Default method from Ogre. Not used currently. Resource listeners can be applied to react on changes in the resource system. */
	virtual void loadResources(void);
	/**< Load all resources that were parsed in 'setupResources' */
	virtual bool frameStarted(const Ogre::FrameEvent& evt);	
	/**< Started before the rendering-process handles the current frame. Used to update the Video stream and eventually record Snapshots. */
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	/**< Started once the render system is processing, the mean-time is used to update the overlay-panels and to process the GAME logic. */
	virtual bool frameEnded(const Ogre::FrameEvent& evt);
	/**< Started when the frame processing is nearly finished. Used to balance the frame rate. */
	
	//Keyboard Listener:
	virtual bool keyPressed( const OIS::KeyEvent &arg );	/**< React on key-pressed events (OIS handler). */
	virtual bool keyReleased( const OIS::KeyEvent &arg );	/**< React on key-release events (OIS handler). */
	//Mouse Listener:
	virtual bool mouseMoved( const OIS::MouseEvent &arg );								/**< React on mouse-moved events (OIS handler). */
	virtual bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );		/**< React on mouse-button-pressed events (OIS handler). */
	virtual bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );	/**< React on mouse-button-released events (OIS handler). */

	//ROS system and callbacks
	virtual void initROS();
	/**< Initialize the ROS System. Configure all subscribers, method synchronization and action clients and register the corresponding callbacks. This method initializes as well an AsyncSpinner with a single independent message thread. */
	virtual void destroyROS();
	/**< Clean up the various ROS pointers (node-handle, subscribers, clients,...) */
	//~ virtual void triggerPanoramaPTUScan(); /**< Do exactly what name suggests! */
	virtual void joyCallback(const sensor_msgs::Joy::ConstPtr& );
	/**< Handle the Joy::ConstPtr messages from the ROS system. The message is checked for the state of the interesting triggers and buttons and communicates the resulting actions to e.g. the PlayerBody class. */
	virtual void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& );
	/**< Receive the 2D ground map and make it available for rendering. Unpacks the msg. stores it in the corresponding Ogre::Image, unsubscribes from the topic and sets the flag for map arrival. */
	//~ virtual void syncCallback(const sensor_msgs::CompressedImageConstPtr&, const sensor_msgs::CompressedImageConstPtr&);  	/**< Synchronized message processing for the room-sweeps. */
	virtual void syncVideoCallback(const sensor_msgs::CompressedImageConstPtr&, const sensor_msgs::CompressedImageConstPtr&);
	/**< Synchronized message processing for the depth-rgb messages of the video-stream (used for snapshots as well). Smoothing of the arrived depth image, 
	color transformation of the rgb image and mapping of the camera transformation into Ogre coordinates. Signalizes the arrival of a video update with a boolean flag. */
	
	// Callbacks added for the demo game:
	virtual void topoNodesCB(const visualization_msgs::InteractiveMarkerInit::ConstPtr& );
	/**< Receive the waypoint markers from ROS, extract and set the required parameters for the Game. */
	virtual void closestWayPointCB(const std_msgs::String::ConstPtr& );
	/**< Update the closest waypoint (or current waypoint) the Robot is located at. */
 
	//Adjust mouse clipping area
	virtual void windowResized(Ogre::RenderWindow* rw);	
	/**< Adjust the clipping area, if the application is running in a window and is resized. */
	//Unattach OIS before window shutdown (very important under Linux)
	virtual void windowClosed(Ogre::RenderWindow* rw);
	/**< Detach the OIS system before the engine shuts down, in order to make the devices available for other applications again. */

	Ogre::Root *mRoot;				/**< The OGRE root. */
	Ogre::SceneManager* mSceneMgr;	/**< The scene manager. */
	Ogre::RenderWindow* mWindow;	/**< The application window. */
	Ogre::String mResourcesCfg;		/**< Path/Name of the resources.cfg file. */
	Ogre::String mPluginsCfg;		/**< Path/Name of the plugins.cfg file. */

	Ogre::OverlaySystem *mOverlaySystem;	/**< Overlay System to display info-panels. */
 
	// OgreBites
	OgreBites::InputContext mInputContext;	/**< Registers the inputs from OIS. */
	OgreBites::SdkTrayManager* mTrayMgr;	/**< The tray manager (for the panels). */
	OgreBites::ParamsPanel* mDetailsPanel;  /**< Panel for the details. */
	bool mCursorWasVisible;					/**< Was the cursor visible before the Dialog was shown. */
	bool mShutDown;							/**< Shut down all systems (broadcasted by this flag). */
 
	//OIS Input devices
	OIS::InputManager* mInputManager;		/**< The input manager for the OIS devices. */
	OIS::Mouse*    mMouse;					/**< The default mouse. */
	OIS::Keyboard* mKeyboard;				/**< The default keyboard. */
	PlayerBody* mPlayer;					/**< Member of the PlayerBody class (part of the src). */
	Ogre::SceneNode *mPlayerBodyNode;		/**< SceneNode to keep track of the player position. */

	//Oculus Rift Support (Kojack)
	Oculus* oculus;							/**< Handler for the OCULUS RIFT (SDK1), by community member Kojack (see Ogre Forum). */

	//ROS connection
	Client *rosPTUClient;					/**< ROS action-client for the PTU commands. */
	boost::thread *ptuSweep;				/**< Thread to launch the PTU sweeps without blocking the rendering. */
	ros::AsyncSpinner* hRosSpinner;			/**< ROS AsyncSpinner, will start the message handling in a separate thread. */
	ros::NodeHandle* hRosNode;				/**< ROS node handle, necessary to run this application as a ros node. */
    ros::Subscriber *hRosSubJoy,			/**< Subscriber for the joystick topic. */
					*hRosSubMap,			/**< Subscriber for the map topic. */
					*hRosSubNodes,			/**< Subscriber for the waypoints. */
					*hRosSubCloseWP;		/**< Subscriber for the closest/actual waypoint. */
	message_filters::Subscriber<sensor_msgs::CompressedImage> 	*hRosSubRGB,		/**< Message filtering to be able to synchronize the image streams. */
																*hRosSubDepth,		/**< Message filtering to be able to synchronize the image streams. */
																*hRosSubRGBVid,		/**< Message filtering to be able to synchronize the image streams. */
																*hRosSubDepthVid;	/**< Message filtering to be able to synchronize the image streams. */
	message_filters::Synchronizer<ApproximateTimePolicy> *rosMsgSync,		/**< Synchronization of the room sweep images. */
															*rosVideoSync;		/**< Synchronization of the video image streams. */
	tf::TransformListener *tfListener;		/**< Keeps track of all coordinate frames. Enables application to compute arbitrary transformations between ROS coordinate frames (see frames.pdf). */
	Robot *robotModel;						/**< Display and manage the robot avatar. */
	GlobalMap *globalMap;					/**< Display and manage the global map. */

	Ogre::Image depImage,	/**< Image to transfer the incomming depth image (room sweep) into the rendering thread. */
				texImage,	/**< Image to transfer the incomming rgb image (room sweep) into the rendering thread. */
				depVideo,	/**< Image to transfer the incomming depth image (video) into the rendering thread. */
				texVideo,	/**< Image to transfer the incomming rgb image (video) into the rendering thread. */
				mapImage; 	/**< Image to transfer the incomming messages into the rendering thread. */
	cv::Mat cv_depth,		/**< OpenCV image (cv::Mat) for preprocessing of the incomming depth image (video, smoothing). */
			cv_rgb;			/**< OpenCV image (cv::Mat) for preprocessing of the incomming rgb image (video, color transformation). */
	SnapshotLibrary *snLib,	/**< Stores manually recorded Snapshots (part of the src). */
					*rsLib;	/**< Stores prerecorded Snapshots (part of the src). */
	Video3D *vdVideo;		/**< Manages the live-feed from the kinect-like camera on the robot. */
	Ogre::Vector3 	snPos,	/**< Vector to transfer the position of incomming (synchronized) image messages from the room sweep. */
					vdPos;	/**< Vector to transfer the position of incomming (synchronized) image messages from the video stream. */
	Ogre::Quaternion 	snOri,	/**< Quaternion to transfer the orientation on incomming (synchronized) image messages from the room sweep. */
						vdOri;	/**< Quaternion to transfer the orientation on incomming (synchronized) image messages from the video stream. */
	volatile bool 	syncedUpdate,	/**< Flag to communicate the arrival of a (synchronized) image update between message and rendering thread (room sweep). */
					videoUpdate,	/**< Flag to communicate the arrival of a (synchronized) image update between message and rendering thread (video stream). */
					takeSnapshot,	/**< Indicator that a snapshot is requested. */
					mapArrived,		/**< Flag to communicate the arrival of the map between message and rendering thread. */
					receivedWPs;	/**< Flag to communicate the arrival of the waypoints between message and rendering thread. */
	
	// for the game	
	SceneNode* cursor;			/**< Added for the Game. Holds the cursor, which is projected onto the ground from the Oculus Rift position and orientation. */
	WayPoint* selectedWP;		/**< The currently selected waypoint. */
	Ogre::String targetWPName;	/**< The name of the waypoint that was selected as a navigation target. */
	int closestWP; 				/**< ID of the closest waypoint (or current waypoint, depending on the subscription). */
	actionlib::SimpleActionClient<topological_navigation::GotoNodeAction> *rosieActionClient; /**< Client for the communication of navigation targets. */
	boost::recursive_mutex GAME_MUTEX;	/**< Mutex to block collision between incomming waypoint update and rendering thread accessing GAME parameters. */

	// Added for Mac compatibility
	Ogre::String m_ResourcePath;	/**< Additional path variable for Mac compatibility. */
 
#ifdef OGRE_STATIC_LIB
	Ogre::StaticPluginLoader m_StaticPluginLoader; /**< Static Plugin Loader. (Ogre default). */
#endif
};
 
#endif // #ifndef __BaseApplication_h_
