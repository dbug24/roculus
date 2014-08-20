/*
-----------------------------------------------------------------------------
Filename:    BaseApplication.cpp
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
#include "BaseApplication.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <stdio.h>
#include <exception>
#include <math.h>

#include <fstream>
using namespace std;

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
#include <macUtils.h>
#endif
 
//-------------------------------------------------------------------------------------
BaseApplication::BaseApplication(void)
	: mRoot(0),
	  mSceneMgr(0),
	  mWindow(0),
	  mResourcesCfg(Ogre::StringUtil::BLANK),
	  mPluginsCfg(Ogre::StringUtil::BLANK),
	  mTrayMgr(0),
	  mDetailsPanel(0),
	  mCursorWasVisible(false),
	  mShutDown(false),
	  mInputManager(0),
	  mMouse(0),
	  mKeyboard(0),
	  mPlayer(0),
	  mPlayerBodyNode(0),
	  mOverlaySystem(0),
	  robotModel(0),
      syncedUpdate(false),
	  takeSnapshot(false),
	  videoUpdate(false),
	  mapArrived(false),
	  receivedWPs(false),
	  closestWP(-1),
	  snPos(Ogre::Vector3::ZERO),
	  snOri(Ogre::Quaternion::IDENTITY),
	  vdPos(Ogre::Vector3::ZERO),
	  vdOri(Ogre::Quaternion::IDENTITY),
	  hRosSubJoy(NULL),
	  hRosSubMap(NULL),
	  hRosSubRGB(NULL),
	  hRosSubDepth(NULL),
	  hRosSubNodes(NULL),
	  hRosSubCloseWP(NULL),
	  rosMsgSync(NULL),
	  rosPTUClient(NULL),
	  ptuSweep(NULL),
	  globalMap(NULL),
	  rosieActionClient(NULL),
	  targetWPName(Ogre::StringUtil::BLANK),
	  selectedWP(NULL)
{
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
    m_ResourcePath = Ogre::macBundlePath() + "/Contents/Resources/";
#else
    m_ResourcePath = "";
#endif
}
 
//-------------------------------------------------------------------------------------
BaseApplication::~BaseApplication(void)
{
	// clean up all rendering related components and managers
	if (mTrayMgr) delete mTrayMgr;
	if (mOverlaySystem) delete mOverlaySystem;
	if (snLib) delete snLib;
	if (rsLib) delete rsLib;
	if (globalMap) delete globalMap;

	//Remove ourself as a Window listener
	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
	windowClosed(mWindow);
	// final engine shutdown
	delete mRoot;
}
 
//-------------------------------------------------------------------------------------
bool BaseApplication::configure(void)
{
	// Show the configuration dialog and initialise the system
	// You can skip this and use root.restoreConfig() to load configuration
	// settings if you were sure there are valid ones saved in ogre.cfg
	if(mRoot->restoreConfig() || mRoot->showConfigDialog())
	{
		// If returned true, user clicked OK so initialise
		// Here we choose to let the system create a default rendering window by passing 'true'
		mWindow = mRoot->initialise(true, "Roculus Render Window");
 
		return true;
	}
	else
	{
		return false;
	}
}
//-------------------------------------------------------------------------------------
void BaseApplication::chooseSceneManager(void)
{
	// Get the SceneManager, in this case a generic one
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

	// initialize the OverlaySystem (changed for 1.9)
	mOverlaySystem = new Ogre::OverlaySystem();
	mSceneMgr->addRenderQueueListener(mOverlaySystem);
}
//-------------------------------------------------------------------------------------
void BaseApplication::createFrameListener(void)
{
	// Set up the management for the input handling
	Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
	OIS::ParamList pl;
	size_t windowHnd = 0;
	std::ostringstream windowHndStr;
 
	mWindow->getCustomAttribute("WINDOW", &windowHnd);
	windowHndStr << windowHnd;
	pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
 
	mInputManager = OIS::InputManager::createInputSystem( pl );
 
	mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, true ));
	mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, true ));

	mMouse->setEventCallback(this);
	mKeyboard->setEventCallback(this);
 
	//Set initial mouse clipping size
	windowResized(mWindow);
 
	//Register as a Window listener
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);
 
	mInputContext.mKeyboard = mKeyboard;
	mInputContext.mMouse = mMouse;
	mTrayMgr = new OgreBites::SdkTrayManager("InterfaceName", mWindow, mInputContext, this);
	mTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
	mTrayMgr->showLogo(OgreBites::TL_BOTTOMRIGHT);
	mTrayMgr->hideCursor();
 
	// create a params panel for displaying sample details
	Ogre::StringVector items;
	items.push_back("cam.pX");
	items.push_back("cam.pY");
	items.push_back("cam.pZ");

	items.push_back("");
	items.push_back("Filtering");
	items.push_back("Poly Mode");
	items.push_back("closestWP");
	items.push_back("GameState");
 
	mDetailsPanel = mTrayMgr->createParamsPanel(OgreBites::TL_NONE, "DetailsPanel", 250, items);
	mDetailsPanel->setParamValue(4, "vertexColors.material");
	mDetailsPanel->setParamValue(5, "Solid (default)");
	mDetailsPanel->hide();
 
	// register at the Ogre root
	mRoot->addFrameListener(this);
}
//-------------------------------------------------------------------------------------
void BaseApplication::destroyScene(void)
{
	if (robotModel) delete robotModel;
	robotModel = NULL;
	if (mPlayer) delete mPlayer;
	mPlayer = NULL;
	if (oculus) delete oculus;
	oculus = NULL;
	// add stuff here if necessary
}
//-------------------------------------------------------------------------------------

void BaseApplication::setupResources(void)
{
	// Load resource paths from config file
	Ogre::ConfigFile cf;
	cf.load(mResourcesCfg);
 
	// Go through all sections & settings in the file
	Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
 
	Ogre::String secName, typeName, archName;
	while (seci.hasMoreElements())
	{
		secName = seci.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator i;
		for (i = settings->begin(); i != settings->end(); ++i)
		{
			typeName = i->first;
			archName = i->second;
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
			// OS X does not set the working directory relative to the app,
			// In order to make things portable on OS X we need to provide
			// the loading with it's own bundle path location
			if (!Ogre::StringUtil::startsWith(archName, "/", false)) // only adjust relative dirs
				archName = Ogre::String(Ogre::macBundlePath() + "/" + archName);
#endif
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
				archName, typeName, secName);
		}
	}
}
//-------------------------------------------------------------------------------------
void BaseApplication::createResourceListener(void)
{
}
//-------------------------------------------------------------------------------------
void BaseApplication::loadResources(void)
{
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}
//-------------------------------------------------------------------------------------
void BaseApplication::go(void)
{
	// These are the resources to be loaded:
	#ifdef _DEBUG
		#ifndef OGRE_STATIC_LIB	
			mResourcesCfg = m_ResourcePath + "resources_d.cfg";
			mPluginsCfg = m_ResourcePath + "plugins_d.cfg";
		#else
			mResourcesCfg = "resources_d.cfg";
			mPluginsCfg = "plugins_d.cfg";
		#endif
	#else
		#ifndef OGRE_STATIC_LIB
			mResourcesCfg = m_ResourcePath + "resources.cfg";
			mPluginsCfg = m_ResourcePath + "plugins.cfg";
		#else
			mResourcesCfg = "resources.cfg";
			mPluginsCfg = "plugins.cfg";
		#endif
	#endif
 
	// set everything up
	if (!setup())
		return;

	// on success ROS can be started
	hRosSpinner->start();
	std::cout << " ---> ROS spinning." << std::endl;
	
	/* wait a second to (hopefully) prevent the nasty thread-collision that keeps shutting down the engine */
	boost::posix_time::milliseconds wait(1000);
	boost::this_thread::sleep(wait);
	
	// start the rendering loop
	mRoot->startRendering();

	// on shutdown stop ROS
	hRosSpinner->stop();
	// clean up scene components
	destroyScene();
	// clean up ROS
	destroyROS();
}

//-------------------------------------------------------------------------------------
bool BaseApplication::setup(void)
{
	// Start the Ogre::Root
	mRoot = new Ogre::Root(mPluginsCfg);
 
	// Parse the resource.cfg file
	setupResources();
 
	bool carryOn = configure();
	if (!carryOn) return false;
 
	chooseSceneManager();

	// Set default mipmap level (NB some APIs ignore this)
	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(3);
 
	// Create any resource listeners (for loading screens)
	createResourceListener();
	// Load resources
	loadResources();
 
	// Create the scene
	createScene();
 	createFrameListener();

	// initialize the GAME (singleton pattern)
	Game::getInstance().init(mSceneMgr);
	
	// get a SceneNode for the PlayerBody
	mPlayerBodyNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("PlayerBodyNode");
	// set up the Oculus Rift (and along with it cameras and viewports of the engine)
	oculus = new Oculus();
	oculus->setupOculus();
	oculus->setupOgre(mSceneMgr, mWindow, mPlayerBodyNode);
	
	// set up the components from the master thesis application
	// (wow, that really was some low level of coding I did there...)
	mPlayer = new PlayerBody(mPlayerBodyNode);
	robotModel = new Robot(mSceneMgr);
	globalMap = new GlobalMap(mSceneMgr);
	
	// configure the ROS setup
	initROS();

	return true;
};
//-------------------------------------------------------------------------------------
bool BaseApplication::frameStarted(const Ogre::FrameEvent& evt)
{
	if(mWindow->isClosed())
		return false;
 
	if(mShutDown)
		return false;
	
	//~ if (syncedUpdate) {
		//~ rsLib->placeInScene(depImage, texImage, snPos, snOri);
		//~ syncedUpdate = false;
	//~ }
	
	// update video node if necessary
	if (videoUpdate) {
		// but first take a Snapshot, if it was requested
		if (takeSnapshot) {
			snLib->placeInScene(depVideo, texVideo, vdPos, vdOri);
			takeSnapshot = false;
		}
		vdVideo->update(depVideo, texVideo, vdPos, vdOri);
		videoUpdate = false;
	}
	
	// insert the map
	if (mapArrived) {
		globalMap->includeMap(mapImage);
		//~ globalMap->flipVisibility();
		mapArrived = false;
	}
	
	return true;
}

bool BaseApplication::frameRenderingQueued(const Ogre::FrameEvent& evt) {
	
	// get the exclusive access to the GAME data
	boost::recursive_mutex::scoped_lock lock(GAME_MUTEX);
	//Need to capture/update each device, JoyStick is handled by ROS
	mKeyboard->capture();
	mMouse->capture();

	mTrayMgr->frameRenderingQueued(evt);

	robotModel->updateFrom(tfListener); // Update the robot's position and orientation
	if (mPlayer->isFirstPerson()) {
		mPlayer->frameRenderingQueued(robotModel);  // first-person mode will mout the player on top of the robot
	} else {
		mPlayer->frameRenderingQueued(evt); // Apply player(~body) movement, if not in first-person
	}

	// update the panel information
	if (mDetailsPanel->isVisible()) {
		mDetailsPanel->setParamValue(0, Ogre::StringConverter::toString(oculus->getCameraNode()->_getDerivedPosition().x));
		mDetailsPanel->setParamValue(1, Ogre::StringConverter::toString(oculus->getCameraNode()->_getDerivedPosition().y));
		mDetailsPanel->setParamValue(2, Ogre::StringConverter::toString(oculus->getCameraNode()->_getDerivedPosition().z));
		mDetailsPanel->setParamValue(6, boost::lexical_cast<std::string>(closestWP));
		mDetailsPanel->setParamValue(7, Game::getInstance().getState());
	}

	// update the oculus orientation
	oculus->update();

	// for game navigation:
	// 1st: update the cursor
	Ogre::Vector3 pos(mPlayerBodyNode->getPosition()+Ogre::Vector3::UNIT_Y*0.7);
	Ogre::Quaternion qView = Ogre::Quaternion(mPlayerBodyNode->getOrientation().getYaw(), Ogre::Vector3::UNIT_Y)*oculus->getOrientation();
	Ogre::Vector3 view(-qView.zAxis());
	if (view.y >= -0.05f) view.y = -0.05f;
	Ogre::Vector3 xzPoint = pos - view*(pos.y/(view.y-0.05f))*0.35f;
	xzPoint.y = 0.05f;
	cursor->setPosition(xzPoint);
	targetWPName = Game::getInstance().highlightClosestWP(cursor->getPosition());
	// 2nd: process the input and derive the GAME's state
	if (Game::getInstance().isRunning() && closestWP != -1) {
		Game::getInstance().frameEventQueued(closestWP);
	}
	
	return true;
}

bool BaseApplication::frameEnded(const Ogre::FrameEvent& evt) {
	// Lock the framerate and save some processing power
	int dt = 25000 - int(1000000.0*evt.timeSinceLastFrame);
	// ...IF we have the resources...
	if (dt < 0) dt = 0;
	if (dt > 25000) dt = 25000;
	boost::posix_time::microseconds wait(dt);
	boost::this_thread::sleep(wait);
	return true;
}

//---------------------------Event Management------------------------------------------
bool BaseApplication::keyPressed( const OIS::KeyEvent &arg )
{
	if (mTrayMgr->isDialogVisible()) return true;   // don't process any more keys if dialog is up
 
	if (arg.key == OIS::KC_F)   // toggle visibility of advanced frame stats
	{
		mTrayMgr->toggleAdvancedFrameStats();
	}
	else if (arg.key == OIS::KC_G)   // toggle visibility of even rarer debugging details
	{
		if (mDetailsPanel->getTrayLocation() == OgreBites::TL_NONE)
		{
			mTrayMgr->moveWidgetToTray(mDetailsPanel, OgreBites::TL_TOPRIGHT, 0);
			mDetailsPanel->show();
		}
		else
		{
			mTrayMgr->removeWidgetFromTray(mDetailsPanel);
			mDetailsPanel->hide();
		}
	} else if (arg.key == OIS::KC_T) { // take a snapshot
		//~ takeSnapshot = true;
	}
	else if(arg.key == OIS::KC_F5)   // refresh all textures
	{
		Ogre::TextureManager::getSingleton().reloadAll();
	}
    else if(arg.key == OIS::KC_P)   // refresh all textures
    {
        mPlayer->toggleFirstPersonMode();
    }
    else if(arg.key == OIS::KC_M) {  // toggle map visibility
		globalMap->flipVisibility();
	}
	else if (arg.key == OIS::KC_SYSRQ)   // take a screenshot
	{
		mWindow->writeContentsToTimestampedFile("screenshot", ".jpg");
	}
	else if (arg.key == OIS::KC_ESCAPE)
	{
		mShutDown = true;
	}
	/* keys added for the GAME */
	else if (arg.key == OIS::KC_SPACE) {
		if (targetWPName != Ogre::StringUtil::BLANK && NULL != rosieActionClient) {
			Game::getInstance().placePersistentMarker(targetWPName);
			rosieActionClient->waitForServer();
			topological_navigation::GotoNodeGoal goal;
			goal.target = targetWPName;
			rosieActionClient->cancelAllGoals();
			rosieActionClient->sendGoal(goal);
			LogManager::getSingletonPtr()->logMessage("SendGoal: " + targetWPName);
			//~ LogManager::getSingletonPtr()->logMessage("!!!I am not sending any goals right now!!!");
		}
	} else if (arg.key == OIS::KC_I) {
		
		/* Code to reinitiate game with the robot driving the the initWP */
		//~ rosieActionClient->waitForServer();
		//~ topological_navigation::GotoNodeGoal goal;
		//~ goal.target = Game::getInstance().getInitWP();
		//~ rosieActionClient->cancelAllGoals();
		//~ rosieActionClient->sendGoal(goal);
		//~ LogManager::getSingletonPtr()->logMessage("Send2InitNode: " + goal.target);
		
		/* Reinitiate the GAME */
		Game::getInstance().startGameSession();
	}

	mPlayer->injectKeyDown(arg);
	return true;
}
 
bool BaseApplication::keyReleased( const OIS::KeyEvent &arg )
{
  mPlayer->injectKeyUp(arg);
  return true;
}
 
bool BaseApplication::mouseMoved( const OIS::MouseEvent &arg )
{
    if (mTrayMgr->injectMouseMove(arg)) return true;
	return true;
}
 
bool BaseApplication::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    if (mTrayMgr->injectMouseDown(arg, id)) return true;
	return true;
}
 
bool BaseApplication::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    if (mTrayMgr->injectMouseUp(arg, id)) return true;
	return true;
}
 
void BaseApplication::windowResized(Ogre::RenderWindow* rw)
{
	//Adjust mouse clipping area
	unsigned int width, height, depth;
	int left, top;
	rw->getMetrics(width, height, depth, left, top);
 
	const OIS::MouseState &ms = mMouse->getMouseState();
	ms.width = width;
	ms.height = height;
}
 

void BaseApplication::windowClosed(Ogre::RenderWindow* rw)
{
	//Unattach OIS before window shutdown (very important under Linux)
	//Only close for window that created OIS (the main window in these demos)
	if( rw == mWindow )
	{
		if( mInputManager )
		{
			mInputManager->destroyInputObject( mMouse );
			mInputManager->destroyInputObject( mKeyboard );
			OIS::InputManager::destroyInputSystem(mInputManager);
			mInputManager = 0;
		}
	}

	
}

//~ void BaseApplication::triggerPanoramaPTUScan() {
	//~ if (rosPTUClient) {
		//~ rosPTUClient->waitForServer();
		//~ scitos_ptu::PanTiltGoal goal;
		//~ // Fill in goal here
		//~ goal.pan_start= -150;
		//~ goal.pan_end= 151;
		//~ goal.pan_step= 50;
		//~ goal.tilt_start= -20;	// OR: -30
		//~ goal.tilt_step= 40;		// 30
		//~ goal.tilt_end= 21;		// 31
		//~ rosPTUClient->sendGoal(goal);
		//~ rosPTUClient->waitForResult(ros::Duration(120.0));
		//~ if (rosPTUClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			//~ Ogre::LogManager::getSingletonPtr()->logMessage("Successfully triggered room panorama.");
		//~ } else {
			//~ Ogre::LogManager::getSingletonPtr()->logMessage("[ERR] Failed to trigger room panorama (!)");
		//~ }
	//~ } else {
		//~ Ogre::LogManager::getSingletonPtr()->logMessage("[ERR] Client not set (!)");
	//~ }
//~ }


/*
  Connection to the ROS system
  -> ROS callback for message handling (asynchronously)
  -> init for setup
  -> destroy for cleanup
*/

//~ void BaseApplication::syncCallback(const sensor_msgs::CompressedImageConstPtr& depthImg, const sensor_msgs::CompressedImageConstPtr& rgbImg) {
//~ 
	//~ if (!syncedUpdate) {
//~ 
		/* DEPRECATED CODE VERSION: Take a look at syncVideoCallback to see how to do it better!!! */
		//~ Ogre::MemoryDataStream depthStr(depthImg->data.size(), false, false);
		//~ Ogre::MemoryDataStream rgbStr(rgbImg->data.size(), false, false);
		//~ 
		//~ Ogre::uint8 buffer1, buffer2, buffer3, buffer4;
		//~ for (int d=0; d<depthImg->data.size(); d++) {
			//~ buffer1 = depthImg->data[d];
			//~ buffer2 = depthImg->data[d+1];
			//~ buffer3 = depthImg->data[d+2];
			//~ buffer4 = depthImg->data[d+3];
			//~ if (buffer1 == 137 && buffer2 == 80 && buffer3 == 78 && buffer4 == 71) {
				//~ depthStr.write(&depthImg->data[d],depthImg->data.size()-d);
				//~ break;
			//~ }
		//~ }
		//~ depthStr.seek(0); //Reset stream position
		//~ Ogre::DataStreamPtr *pDstr = new Ogre::DataStreamPtr(&depthStr);
		//~ 
		//~ for (int d=0; d<rgbImg->data.size(); d++) {
			//~ buffer1 = rgbImg->data[d];
			//~ buffer2 = rgbImg->data[d+1];
			//~ if (buffer1 == 255 && buffer2 == 216)  {
				//~ rgbStr.write(&rgbImg->data[d], rgbImg->data.size()-d);
				//~ break;
			//~ }
		//~ }
		//~ rgbStr.seek(0); // Reset stream position
		//~ Ogre::DataStreamPtr *pRstr = new Ogre::DataStreamPtr(&rgbStr);
		//~ 
		//~ using namespace Ogre;
		//~ static tf::StampedTransform snTransform;
		//~ static tfScalar yaw,pitch,roll;
		//~ static Matrix3 mRot;
		//~ 
		//~ try {
			//~ tfListener->lookupTransform("map", "head_xtion_depth_optical_frame", depthImg->header.stamp, snTransform);
			//~ 
			//~ snPos.x = -snTransform.getOrigin().y();
			//~ snPos.y = snTransform.getOrigin().z();
			//~ snPos.z = -snTransform.getOrigin().x();
			//~ 
			//~ snTransform.getBasis().getEulerYPR(yaw,pitch,roll);
			//~ mRot.FromEulerAnglesXYZ(-Radian(pitch),Radian(yaw),-Radian(roll));
			//~ snOri.FromRotationMatrix(mRot);
			//~ 
			//~ depImage.load(*pDstr, "png");
			//~ texImage.load(*pRstr, "jpeg");
//~ 
			//~ syncedUpdate = true;
		//~ } catch (tf::TransformException ex) {
			//~ ROS_ERROR("%s",ex.what());
		//~ }		
	//~ }
//~ }

void BaseApplication::syncVideoCallback(const sensor_msgs::CompressedImageConstPtr& depthImg, const sensor_msgs::CompressedImageConstPtr& rgbImg) {
	/* Receive a depth-rgb pair of images, filter and convert them into the Ogre format and fetch the according transformation
	 * in order to complete a valid Snapshot */
	 
	 // used for the coordinate tranformation from ROS to Ogre
	static tfScalar yaw,pitch,roll;
	static tf::StampedTransform vdTransform;
	static Ogre::Matrix3 mRot;
	
	 // Only if the last update was rendered
	if (!videoUpdate) {			
		try {
			// We have to cut away the compression header to load the depth image into openCV
			compressed_depth_image_transport::ConfigHeader compressionConfig;
			memcpy(&compressionConfig, &depthImg->data[0], sizeof(compressionConfig));
			const std::vector<uint8_t> depthData(depthImg->data.begin() + sizeof(compressionConfig), depthImg->data.end());
			
			// load the images:
			cv::Mat tmp_depth = cv::imdecode(cv::Mat(depthData), CV_LOAD_IMAGE_UNCHANGED);
			cv::Mat tmp_rgb = cv::imdecode(cv::Mat(rgbImg->data), CV_LOAD_IMAGE_UNCHANGED);
			tmp_depth.convertTo(cv_depth, CV_16U);
			tmp_rgb.convertTo(cv_rgb, CV_8UC3);
			
			// process images, by bluring the depth and rearranging the color values
			cv::GaussianBlur(cv_depth, cv_depth, cv::Size(11,11), 0, 0);
			cv::cvtColor(cv_rgb,cv_rgb, CV_BGR2RGB);

			/* lookup the transform and convert them to the OGRE coordinates
			 *  unfortunately there is still some magic going on in Video3D.cpp and Snapshot.cpp
			 *  in order to end up in the correct orientation...
			*/
			tfListener->lookupTransform("map", "head_xtion_depth_optical_frame", depthImg->header.stamp, vdTransform);
			
			// positioning
			vdPos.x = -vdTransform.getOrigin().y();
			vdPos.y = vdTransform.getOrigin().z();
			vdPos.z = -vdTransform.getOrigin().x();
			
			// rotation (at least get it into global coords that are fixed on the robot)
			vdTransform.getBasis().getEulerYPR(yaw,pitch,roll);
			mRot.FromEulerAnglesXYZ(-Radian(pitch),Radian(yaw),-Radian(roll));
			vdOri.FromRotationMatrix(mRot);
		
			/* connect the data to the images, note that this does in fact not load, but store pointers instead
			 * which is why the cv_depth and cv_rgb are members of BaseApplication to prevent data loss
			*/
			depVideo.loadDynamicImage(static_cast<uchar*>(cv_depth.data), cv_depth.cols, cv_depth.rows, 1, Ogre::PF_L16);
			texVideo.loadDynamicImage(static_cast<uchar*>(cv_rgb.data), cv_rgb.cols, cv_rgb.rows, 1, Ogre::PF_BYTE_RGB);
			videoUpdate = true;
			
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
			videoUpdate = false;
		} catch (std::exception& e) {
			std::cerr << e.what() << std::endl;
			videoUpdate = false;
		}
	}
}

void BaseApplication::joyCallback(const sensor_msgs::Joy::ConstPtr &joy ) {
	/*
	 * the static variables prevent jitter and repetitive commands
	 * */
	static bool l_button0 = false; 
	static bool l_button1 = false;
	static bool l_button2 = false;
	static bool l_button3 = false;
	static bool l_button5 = false;
	static bool l_button9 = false;
	
	// pass input on to player movements
	mPlayer->injectROSJoy(joy);
	
	if (l_button0 == false && joy->buttons[0] != 0 && takeSnapshot == false) {
		// request recording of a Snapshot
		takeSnapshot = true;
	}
	else if (l_button1 == false && joy->buttons[1] != 0) {
		// make all manually taken Snapshots invisible (effectively you can record a second set of images)
		snLib->flipVisibility();
	}
	else if (l_button2 == false && joy->buttons[2] != 0) {
		// make all room sweep Snapshots invisible (effectively you can record a second set of images)
		rsLib->flipVisibility();
	}
	else if (l_button3 == false && joy->buttons[3] != 0) {
		// trigger a room sweep
		//~ boost::thread tmpThread(boost::bind(&BaseApplication::triggerPanoramaPTUScan, this));
	}
	else if (l_button5 == false && joy->buttons[5] != 0) {
		// switch between first person and free viewpoint
		mPlayer->toggleFirstPersonMode();
	}
	else if (joy->buttons[7] != 0) {
		// set the oculus orientation back to IDENTITY (effectively looking into the direction the PlayerBody has)
		oculus->resetOrientation();
	}
	else if (l_button9 == false && joy->buttons[9] != 0) {
		// toggle the map
		globalMap->flipVisibility();
	}
	
	// make sure that button is released before triggering an event repeatedly
	l_button0 = (joy->buttons[0] != 0);
	l_button1 = (joy->buttons[1] != 0);
	l_button2 = (joy->buttons[2] != 0);
	l_button3 = (joy->buttons[3] != 0);
	l_button5 = (joy->buttons[5] != 0);
	l_button9 = (joy->buttons[9] != 0);
}

void BaseApplication::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
	/* Still the old code version with a memory leak (but is only triggered once) */
	Ogre::MemoryDataStream dataStream(map->data.size(), false, false);
	Ogre::uint8 value(0);
	for (int i=0; i<map->data.size(); i++) {
		if (map->data[i] == -1) {
			value = 40;
		} else {
			value = 2*map->data[i];
		}
		dataStream.write(&value, sizeof(Ogre::uint8));
	}
	dataStream.seek(0);
	
	globalMap->insertWHR(map->info.width, map->info.height, map->info.resolution);
	globalMap->setOrigin(Vector3(map->info.origin.position.y, map->info.origin.position.z, -map->info.origin.position.x));

	Ogre::DataStreamPtr *pMap = new Ogre::DataStreamPtr(&dataStream);
	mapImage.loadRawData(*pMap, map->info.width, map->info.height, Ogre::PF_L8);
	hRosSubMap->shutdown();
	mapArrived = true;
}

void BaseApplication::topoNodesCB(const visualization_msgs::InteractiveMarkerInit::ConstPtr& data){
	// get exclusive access for the GAME in order to reset the waypoint parameters
	boost::recursive_mutex::scoped_lock lock(GAME_MUTEX);

	for (int i=0;i<data->markers.size();i++) {
		WayPoint *wp = Game::getInstance().getWPByName(data->markers[i].name);
		if (wp) {
			wp->setPosition(Vector3(-data->markers[i].pose.position.y,
				data->markers[i].pose.position.z,
				-data->markers[i].pose.position.x));
			Quaternion wpRot(data->markers[i].pose.orientation.w,
				-data->markers[i].pose.orientation.y,
				data->markers[i].pose.orientation.z,
				-data->markers[i].pose.orientation.x);
			if (wpRot != Quaternion::ZERO) {
				wp->setOrientation(wpRot);
			}
			
			wp->setVisible(true);
			Ogre::LogManager::getSingletonPtr()->logMessage(wp->toString() + " arrived.");
		} else {
			cerr << "Inexistant WayPoint transmittet by ROS." << endl;
		}
	}
	
	hRosSubNodes->shutdown();
	//~ Game::getInstance().print();
	receivedWPs = true;	
}

void BaseApplication::closestWayPointCB(const std_msgs::String::ConstPtr& data) {
	// get the waypoint of the robot
	std::string name(data->data);
	if (name.substr(0,8).compare("WayPoint") == 0) {
		closestWP = boost::lexical_cast<int>(name.erase(0,8)); //Erase string "WayPoint" before casting
	} else {
		std::cout << "Node unknown:" << name << std::endl;
		closestWP = -1;
	}
}

void BaseApplication::initROS() {
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "roculus");
  hRosNode = new ros::NodeHandle();
  
  /* Subscribe to the joystick input */
  hRosSubJoy = new ros::Subscriber(hRosNode->subscribe<sensor_msgs::Joy>
				("/joy/visualization", 10, boost::bind(&BaseApplication::joyCallback, this, _1)));
				
  /* Subscribe for the map topic (Published Once per Subscriber) and navigation topics */
  hRosSubMap = new ros::Subscriber(hRosNode->subscribe<nav_msgs::OccupancyGrid>
				("/map", 1, boost::bind(&BaseApplication::mapCallback, this, _1)));
                
  /* Subscription for the waypoints */
  hRosSubNodes = new ros::Subscriber(hRosNode->subscribe<visualization_msgs::InteractiveMarkerInit>
                ("/kth_floorsix_y2_topo_markers/update_full", 1, boost::bind(&BaseApplication::topoNodesCB, this, _1)));
  
  /* Subscription for the closest WP / current WP */
  hRosSubCloseWP = new ros::Subscriber(hRosNode->subscribe<std_msgs::String>
                ("/current_node", 1, boost::bind(&BaseApplication::closestWayPointCB, this, _1)));


	/* Subscription and binding for the 360deg images */
  //~ hRosSubRGB = new message_filters::Subscriber<sensor_msgs::CompressedImage>
				//~ (*hRosNode, "/local_metric_map/rgb/rgb_filtered/compressed", 1);
  //~ hRosSubDepth = new message_filters::Subscriber<sensor_msgs::CompressedImage>
				//~ (*hRosNode, "/local_metric_map/depth/depth_filtered/compressedDepth", 1);
  //~ rosMsgSync = new message_filters::Synchronizer<ApproximateTimePolicy>
				//~ (ApproximateTimePolicy(15), *hRosSubDepth, *hRosSubRGB);
  //~ rosMsgSync->registerCallback(boost::bind(&BaseApplication::syncCallback, this, _1, _2));
  
	/* Subscription and binding for the 3D video and requested snapshots */
  hRosSubRGBVid = new message_filters::Subscriber<sensor_msgs::CompressedImage>
				//~ (*hRosNode, "/head_xtion/rgb/image_raw_low_fps/compressed", 1);
				//~ (*hRosNode, "/head_xtion/rgb/image_color/compressed", 1);
				(*hRosNode, "/head_xtion/rgb/image_color/reducedBW/compressed", 1);
  hRosSubDepthVid = new message_filters::Subscriber<sensor_msgs::CompressedImage>
				//~ (*hRosNode, "/head_xtion/depth/image_raw_low_fps/compressedDepth", 1);
				//~ (*hRosNode, "/head_xtion/depth_registered/image_rect/compressedDepth", 1);
				(*hRosNode, "/head_xtion/depth_registered/image_rect/reducedBW/compressedDepth", 1);
  rosVideoSync = new message_filters::Synchronizer<ApproximateTimePolicy>
				(ApproximateTimePolicy(15), *hRosSubDepthVid, *hRosSubRGBVid);
  rosVideoSync->registerCallback(boost::bind(&BaseApplication::syncVideoCallback, this, _1, _2));
  
	/* Setting up the tfListener, action clients and initialize the AsyncSpinner */
  //~ rosPTUClient = new Client("ptu_pan_tilt_metric_map", true);
  rosieActionClient = new actionlib::SimpleActionClient<topological_navigation::GotoNodeAction>("topological_navigation", true);
  tfListener = new tf::TransformListener();
  
  /* AsyncSpinner to process msgs. in a separate thread (param =!= 1) */
  hRosSpinner = new ros::AsyncSpinner(1);
}

void BaseApplication::destroyROS() {
	// shutdown ROS and free all memory, if necessary
  ros::shutdown();
  if (hRosSpinner) {
    delete hRosSpinner;
    hRosSpinner = NULL;
  }
  if (rosMsgSync) {
    delete rosMsgSync;
    rosMsgSync = NULL;
  }
  if (rosVideoSync) {
    delete rosVideoSync;
    rosVideoSync = NULL;
  }
  if (hRosSubJoy) {
	delete hRosSubJoy;
	hRosSubJoy = NULL;
  }
  if (hRosSubMap) {
	delete hRosSubMap;
	hRosSubMap = NULL;
  }
  if (hRosSubRGB) {
    delete hRosSubRGB;
    hRosSubRGB = NULL;
  }
  if (hRosSubDepth) {
    delete hRosSubDepth;
    hRosSubDepth = NULL;
  }
  if (hRosSubRGBVid) {
    delete hRosSubRGB;
    hRosSubRGB = NULL;
  }
  if (hRosSubDepthVid) {
    delete hRosSubDepth;
    hRosSubDepth = NULL;
  }
  if (hRosSubNodes) {
    delete hRosSubNodes;
    hRosSubNodes = NULL;
  }
  if (hRosSubCloseWP) {
    delete hRosSubCloseWP;
    hRosSubCloseWP = NULL;
  }
  if (hRosNode) {
    delete hRosNode;
    hRosNode = NULL;
  }
  if (rosPTUClient) {
	delete rosPTUClient;
	rosPTUClient = NULL;
  }
  if (rosieActionClient) {
	  delete rosieActionClient;
	  rosieActionClient = NULL;
  }
}
