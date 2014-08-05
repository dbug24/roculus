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
	  mCamera(0),
	  mSceneMgr(0),
	  mWindow(0),
	  mResourcesCfg(Ogre::StringUtil::BLANK),
	  mPluginsCfg(Ogre::StringUtil::BLANK),
	  mTrayMgr(0),
	  mCameraMan(0),
	  mDetailsPanel(0),
	  mCursorWasVisible(false),
	  mShutDown(false),
	  mInputManager(0),
	  mMouse(0),
	  mKeyboard(0),
	  mJoyStick(0),
	  mPlayer(0),
	  mPlayerBodyNode(0),
	  mOverlaySystem(0),
	  mPCRender(0),
	  robotModel(0),
      syncedUpdate(false), // was false # Rares
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
      hRosSubPose(NULL),
	  hRosSubRGB(NULL),
	  hRosSubDepth(NULL),
	  hRosSubNodes(NULL),
	  hRosSubCloseWP(NULL),
	  rosMsgSync(NULL),
	  rosPTUClient(NULL),
	  ptuSweep(NULL),
	  globalMap(NULL),
	  demoGame(NULL),
	  rosieActionClient(NULL),
	  selectedWP(NULL)
{
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
    m_ResourcePath = Ogre::macBundlePath() + "/Contents/Resources/";
#else
    m_ResourcePath = "";
#endif

    m_bBufferSnapshotData = false;
}
 
//-------------------------------------------------------------------------------------
BaseApplication::~BaseApplication(void)
{
	if (mTrayMgr) delete mTrayMgr;
	if (mCameraMan) delete mCameraMan;
	if (mOverlaySystem) delete mOverlaySystem;
	if (snLib) delete snLib;
	if (rsLib) delete rsLib;
	if (globalMap) delete globalMap;

	//Remove ourself as a Window listener
	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
	windowClosed(mWindow);
	delete mRoot;
}
 
//-------------------------------------------------------------------------------------
bool BaseApplication::configure(void)
{
	// Show the configuration dialog and initialise the system
	// You can skip this and use root.restoreConfig() to load configuration
	// settings if you were sure there are valid ones saved in ogre.cfg
	if(mRoot->showConfigDialog())
	{
		// If returned true, user clicked OK so initialise
		// Here we choose to let the system create a default rendering window by passing 'true'
		mWindow = mRoot->initialise(true, "TutorialApplication Render Window");
 
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
//---NOT USED FOR ROSCULUS APPLICATION-------------------------------------------------
void BaseApplication::createCamera(void)
{
  	// Create the camera
	mCamera = mSceneMgr->createCamera("PlayerCam");
 
	// Position it at 500 in Z direction
	mCamera->setPosition(Ogre::Vector3(0,0,80));
	// Look back along -Z
	mCamera->lookAt(Ogre::Vector3(0,0,-300));
    mCamera->setNearClipDistance(0.0f);
	mCamera->setFarClipDistance(3000.0f);	

	mCameraMan = new OgreBites::SdkCameraMan(mCamera);   // create a default camera controller
}
//-------------------------------------------------------------------------------------
void BaseApplication::createFrameListener(void)
{
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
	//~ mJoyStick = static_cast<OIS::JoyStick*>(mInputManager->createInputObject( OIS::OISJoyStick, true ));

	mMouse->setEventCallback(this);
	mKeyboard->setEventCallback(this);
	//~ mJoyStick->setEventCallback(this);
 
	//Set initial mouse clipping size
	windowResized(mWindow);
 
	//Register as a Window listener
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);
 
	mInputContext.mKeyboard = mKeyboard;
	mInputContext.mMouse = mMouse;
	//~ mInputContext.mJoyStick = mJoyStick;
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
	items.push_back("cam.oW");
	items.push_back("cam.oX");
	items.push_back("cam.oY");
	items.push_back("cam.oZ");
	items.push_back("");
	items.push_back("Filtering");
	items.push_back("Poly Mode");
	items.push_back("closestWP");
	items.push_back("RobotYaw");
 
	mDetailsPanel = mTrayMgr->createParamsPanel(OgreBites::TL_NONE, "DetailsPanel", 200, items);
	mDetailsPanel->setParamValue(9, "Anisotropic");
	mDetailsPanel->setParamValue(10, "Solid");
	mDetailsPanel->hide();
 
	mRoot->addFrameListener(this);
}
//-------------------------------------------------------------------------------------
void BaseApplication::destroyScene(void)
{
}
//-------------------------------------------------------------------------------------

void BaseApplication::cleanUp(void) {
	delete demoGame;
}

void BaseApplication::createViewports(void)
{
	// Create one viewport, entire window
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0.1f,0.1f,0.1f));
 
	// Alter the camera aspect ratio to match the viewport
	mCamera->setAspectRatio(
		Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
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
 
	if (!setup())
		return;

	hRosSpinner->start();
	std::cout << " ---> ROS spinning." << std::endl;
	mRoot->startRendering();

	hRosSpinner->stop();
	destroyScene();

	if (robotModel) delete robotModel;
	robotModel = NULL;
	if (mPlayer) delete mPlayer;
	mPlayer = NULL;
	if (oculus) delete oculus;
	oculus = NULL;

	destroyROS();
	cleanUp();
}

//-------------------------------------------------------------------------------------
bool BaseApplication::setup(void)
{
	mRoot = new Ogre::Root(mPluginsCfg);
 
	setupResources();
 
	bool carryOn = configure();
	if (!carryOn) return false;
 
	chooseSceneManager();

	// Set default mipmap level (NB some APIs ignore this)
	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
 
	// Create any resource listeners (for loading screens)
	createResourceListener();
	// Load resources
	loadResources();
 
	// Create the scene
	createScene();
 
	createFrameListener();
	
	demoGame = new Game(mSceneMgr);
	//~ demoGame->print();
	
	mPlayerBodyNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	oculus = new Oculus();
	oculus->setupOculus();
	oculus->setupOgre(mSceneMgr, mWindow, mPlayerBodyNode);
	mPlayer = new PlayerBody(mPlayerBodyNode);
	robotModel = new Robot(mSceneMgr);
	globalMap = new GlobalMap(mSceneMgr);
	
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
	
	if (syncedUpdate) {
		rsLib->placeInScene(depImage, texImage, snPos, snOri);
		syncedUpdate = false;

        static int index = 0;

        if (m_bBufferSnapshotData)
        {
            std::cout<<"Saving depth and rgb textures"<<std::endl;
            char filename_rgb[50];
            sprintf(filename_rgb,"snap_rgb%d.jpg",index);
            texImage.save(filename_rgb);

            char filename_depth[50];
            sprintf(filename_depth,"snap_depth%d.png",index);
            depImage.save(filename_depth);

            ofstream myfile;
            char filename_pos[50];
            sprintf(filename_pos,"snap_pos%d.txt",index);
            myfile.open (filename_pos);
            myfile<<Ogre::StringConverter::toString(snPos)<<"\n"<<Ogre::StringConverter::toString(snOri);
            myfile.close();

            index++;
        }
	}
	
	if (videoUpdate) {
		if (takeSnapshot) {
			snLib->placeInScene(depVideo, texVideo, vdPos, vdOri);
			takeSnapshot = false;
		}
		vdVideo->update(texVideo, depVideo, vdPos, vdOri);
		videoUpdate = false;
	}
	
	if (mapArrived) {
		globalMap->includeMap(mapImage);
		globalMap->flipVisibility();
		mapArrived = false;
	}
	
	return true;
}

bool BaseApplication::frameRenderingQueued(const Ogre::FrameEvent& evt) {
	//Need to capture/update each device
	mKeyboard->capture();
	mMouse->capture();
	//~ mJoyStick->capture(); //OIS Joystick was deactivated and replaced by ROS

	mTrayMgr->frameRenderingQueued(evt);

	robotModel->updateFrom(tfListener); // Update the robot as well
	if (mPlayer->isFirstPerson()) {
		mPlayer->frameRenderingQueued(robotModel);  
	} else {
		mPlayer->frameRenderingQueued(evt); // Apply player(~body) movement
	}

	if (mDetailsPanel->isVisible()) {
		mDetailsPanel->setParamValue(0, Ogre::StringConverter::toString(oculus->getCameraNode()->_getDerivedPosition().x));
		mDetailsPanel->setParamValue(1, Ogre::StringConverter::toString(oculus->getCameraNode()->_getDerivedPosition().y));
		mDetailsPanel->setParamValue(2, Ogre::StringConverter::toString(oculus->getCameraNode()->_getDerivedPosition().z));
		mDetailsPanel->setParamValue(4, Ogre::StringConverter::toString(oculus->getCameraNode()->_getDerivedOrientation().w));
		mDetailsPanel->setParamValue(5, Ogre::StringConverter::toString(oculus->getCameraNode()->_getDerivedOrientation().x));
		mDetailsPanel->setParamValue(6, Ogre::StringConverter::toString(oculus->getCameraNode()->_getDerivedOrientation().y));
		mDetailsPanel->setParamValue(7, Ogre::StringConverter::toString(oculus->getCameraNode()->_getDerivedOrientation().z));
		mDetailsPanel->setParamValue(11, boost::lexical_cast<std::string>(closestWP));
		mDetailsPanel->setParamValue(12, Ogre::StringConverter::toString(mPlayerBodyNode->getOrientation().getYaw()));
	}

	oculus->update(); // Set OCULUS orientation

	// For game navigation:
	Ogre::Vector3 pos(mPlayerBodyNode->getPosition()+Ogre::Vector3::UNIT_Y*0.7);
	Ogre::Quaternion qView = Ogre::Quaternion(mPlayerBodyNode->getOrientation().getYaw(), Ogre::Vector3::UNIT_Y)*oculus->getOrientation();
	Ogre::Vector3 view(-qView.zAxis());
	if (view.y >= -0.05f) view.y = -0.05f;
	Ogre::Vector3 xzPoint = pos - view*(pos.y/(view.y-0.05f))*0.35f;
	xzPoint.y = 0.05f;
	cursor->setPosition(xzPoint);
}

bool BaseApplication::frameEnded(const Ogre::FrameEvent& evt) {
	// Lock the framerate and save some processing power
	int dt = 25000 - int(1000000.0*evt.timeSinceLastFrame);
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
	} else if (arg.key == OIS::KC_T) {
		takeSnapshot = true;
	}
	//~ else if (arg.key == OIS::KC_T)   // cycle polygon rendering mode
	//~ {
		//~ Ogre::String newVal;
		//~ Ogre::TextureFilterOptions tfo;
		//~ unsigned int aniso;
 //~ 
		//~ switch (mDetailsPanel->getParamValue(9).asUTF8()[0])
		//~ {
		//~ case 'B':
			//~ newVal = "Trilinear";
			//~ tfo = Ogre::TFO_TRILINEAR;
			//~ aniso = 1;
			//~ break;
		//~ case 'T':
			//~ newVal = "Anisotropic";
			//~ tfo = Ogre::TFO_ANISOTROPIC;
			//~ aniso = 8;
			//~ break;
		//~ case 'A':
			//~ newVal = "None";
			//~ tfo = Ogre::TFO_NONE;
			//~ aniso = 1;
			//~ break;
		//~ default:
			//~ newVal = "Bilinear";
			//~ tfo = Ogre::TFO_BILINEAR;
			//~ aniso = 1;
		//~ }
 //~ 
		//~ Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(tfo);
		//~ Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(aniso);
		//~ mDetailsPanel->setParamValue(9, newVal);
	//~ }
	else if(arg.key == OIS::KC_F5)   // refresh all textures
	{
		Ogre::TextureManager::getSingleton().reloadAll();
        syncedUpdate = true;
        std::cout<<"Starting to receive sweep frames"<<std::endl;
	}
    else if(arg.key == OIS::KC_F6)   // refresh all textures
    {
        m_bBufferSnapshotData = !m_bBufferSnapshotData;

        if (m_bBufferSnapshotData)
        {
            std::cout<<"Saving map"<<std::endl;
        } else {
            std::cout<<"Stopped saving map"<<std::endl;
        }
    }
    else if(arg.key == OIS::KC_F7)   // refresh all textures
    {
        loadSavedMap();
    }
    else if(arg.key == OIS::KC_P)   // refresh all textures
    {
        mPlayer->toggleFirstPersonMode();
    }
    else if(arg.key == OIS::KC_M) {
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
	else if (arg.key == OIS::KC_SPACE) {
		Ogre::String target = demoGame->highlightClosestWP(cursor->getPosition());
		if (target != Ogre::StringUtil::BLANK && NULL != rosieActionClient) {
			rosieActionClient->waitForServer();
			topological_navigation::GotoNodeGoal goal;
			goal.target = target;
			rosieActionClient->cancelAllGoals();
			rosieActionClient->sendGoal(goal);
			LogManager::getSingletonPtr()->logMessage("SendGoal: " + target);
		}
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
 
//Adjust mouse clipping area
void BaseApplication::windowResized(Ogre::RenderWindow* rw)
{
	unsigned int width, height, depth;
	int left, top;
	rw->getMetrics(width, height, depth, left, top);
 
	const OIS::MouseState &ms = mMouse->getMouseState();
	ms.width = width;
	ms.height = height;
}
 
//Unattach OIS before window shutdown (very important under Linux)
void BaseApplication::windowClosed(Ogre::RenderWindow* rw)
{
	//Only close for window that created OIS (the main window in these demos)
	if( rw == mWindow )
	{
		if( mInputManager )
		{
			mInputManager->destroyInputObject( mMouse );
			mInputManager->destroyInputObject( mKeyboard );
			mInputManager->destroyInputObject( mJoyStick );
			OIS::InputManager::destroyInputSystem(mInputManager);
			mInputManager = 0;
		}
	}

	
}

bool BaseApplication::povMoved( const OIS::JoyStickEvent &arg, int pov ) {
  mPlayer->injectPOVChanged(arg, pov);
  return false;
}

bool BaseApplication::axisMoved( const OIS::JoyStickEvent &arg, int axis ) {
  mPlayer->injectAxisMoved(arg, axis);
  return true;
}

bool BaseApplication::sliderMoved( const OIS::JoyStickEvent &arg, int sliderID ) {
  return false;
}

bool BaseApplication::buttonPressed( const OIS::JoyStickEvent &arg, int button ) {
	mPlayer->injectButtonDown(arg, button);
	return true;
}

bool BaseApplication::buttonReleased( const OIS::JoyStickEvent &arg, int button ) {
  mPlayer->injectButtonUp(arg, button);
  return true;
}

void BaseApplication::triggerPanoramaPTUScan() {
	if (rosPTUClient) {
		rosPTUClient->waitForServer();
		scitos_ptu::PanTiltGoal goal;
		// Fill in goal here
		goal.pan_start= -150;
		goal.pan_end= 151;
		goal.pan_step= 50;
		goal.tilt_start= -25;	// OR: -30
		goal.tilt_step= 50;		// 30
		goal.tilt_end= 26;		// 31
		rosPTUClient->sendGoal(goal);
		rosPTUClient->waitForResult(ros::Duration(120.0));
		if (rosPTUClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			Ogre::LogManager::getSingletonPtr()->logMessage("Successfully triggered room panorama.");
		} else {
			Ogre::LogManager::getSingletonPtr()->logMessage("[ERR] Failed to trigger room panorama (!)");
		}
	} else {
		Ogre::LogManager::getSingletonPtr()->logMessage("[ERR] Client not set (!)");
	}
}


/*
  Connection to the ROS system
  -> ROS callback for message handling (asynchronously)
  -> init for setup
  -> destroy for cleanup
*/

void BaseApplication::syncCallback(const sensor_msgs::CompressedImageConstPtr& depthImg, const sensor_msgs::CompressedImageConstPtr& rgbImg) {

	if (!syncedUpdate) {

		Ogre::MemoryDataStream depthStr(depthImg->data.size(), false, false);
		Ogre::MemoryDataStream rgbStr(rgbImg->data.size(), false, false);
		
		Ogre::uint8 buffer1, buffer2, buffer3, buffer4;
		for (int d=0; d<depthImg->data.size(); d++) {
			buffer1 = depthImg->data[d];
			buffer2 = depthImg->data[d+1];
			buffer3 = depthImg->data[d+2];
			buffer4 = depthImg->data[d+3];
			if (buffer1 == 137 && buffer2 == 80 && buffer3 == 78 && buffer4 == 71) {
				depthStr.write(&depthImg->data[d],depthImg->data.size()-d);
				break;
			}
		}
		depthStr.seek(0); //Reset stream position
		Ogre::DataStreamPtr *pDstr = new Ogre::DataStreamPtr(&depthStr);
		
		for (int d=0; d<rgbImg->data.size(); d++) {
			buffer1 = rgbImg->data[d];
			buffer2 = rgbImg->data[d+1];
			if (buffer1 == 255 && buffer2 == 216)  {
				rgbStr.write(&rgbImg->data[d], rgbImg->data.size()-d);
				break;
			}
		}
		rgbStr.seek(0); // Reset stream position
		Ogre::DataStreamPtr *pRstr = new Ogre::DataStreamPtr(&rgbStr);
		
		using namespace Ogre;
		static tfScalar yaw,pitch,roll;
		static Matrix3 mRot;
		
		try {
			tfListener->lookupTransform("map", "head_xtion_depth_optical_frame", depthImg->header.stamp, snTransform);
			
			snPos.x = -snTransform.getOrigin().y();
			snPos.y = snTransform.getOrigin().z();
			snPos.z = -snTransform.getOrigin().x();
			
			snTransform.getBasis().getEulerYPR(yaw,pitch,roll);
			mRot.FromEulerAnglesYXZ(Radian(yaw),Radian(pitch),Radian(roll));
			snOri.FromRotationMatrix(mRot);
			
			depImage.load(*pDstr, "png");
			texImage.load(*pRstr, "jpeg");

			syncedUpdate = true;
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}		
	}
}

void BaseApplication::syncVideoCallback(const sensor_msgs::CompressedImageConstPtr& depthImg, const sensor_msgs::CompressedImageConstPtr& rgbImg) {
	static int cb_cnt = 0;
	cb_cnt++;
	if (!videoUpdate && cb_cnt >= 2) {
		cb_cnt = 0;

		Ogre::MemoryDataStream depthStr(depthImg->data.size(), false, false);
		Ogre::MemoryDataStream rgbStr(rgbImg->data.size(), false, false);
		
		Ogre::uint8 buffer1, buffer2, buffer3, buffer4;
		for (int d=0; d<depthImg->data.size(); d++) {
			buffer1 = depthImg->data[d];
			buffer2 = depthImg->data[d+1];
			buffer3 = depthImg->data[d+2];
			buffer4 = depthImg->data[d+3];
			if (buffer1 == 137 && buffer2 == 80 && buffer3 == 78 && buffer4 == 71) {
				depthStr.write(&depthImg->data[d],depthImg->data.size()-d);
				break;
			}
		}
		depthStr.seek(0); //Reset stream position
		Ogre::DataStreamPtr *pDstr = new Ogre::DataStreamPtr(&depthStr);
		
		for (int d=0; d<rgbImg->data.size(); d++) {
			buffer1 = rgbImg->data[d];
			buffer2 = rgbImg->data[d+1];
			if (buffer1 == 255 && buffer2 == 216)  {
				rgbStr.write(&rgbImg->data[d], rgbImg->data.size()-d);
				break;
			}
		}
		rgbStr.seek(0); // Reset stream position
		Ogre::DataStreamPtr *pRstr = new Ogre::DataStreamPtr(&rgbStr);
		
		using namespace Ogre;
		static tfScalar yaw,pitch,roll;
		static Matrix3 mRot;
		
		try {
			tfListener->lookupTransform("map", "chest_xtion_depth_optical_frame", depthImg->header.stamp, vdTransform);
			
			vdPos.x = -vdTransform.getOrigin().y();
			vdPos.y = vdTransform.getOrigin().z();
			vdPos.z = -vdTransform.getOrigin().x();
			
			vdTransform.getBasis().getEulerYPR(yaw,pitch,roll);
			mRot.FromEulerAnglesYXZ(Radian(yaw),Radian(pitch),Radian(roll));
			vdOri.FromRotationMatrix(mRot);			
		
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}
		
		depVideo.load(*pDstr, "png");
		texVideo.load(*pRstr, "jpeg");

		videoUpdate = true;
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
		takeSnapshot = true;
	}
	else if (l_button1 == false && joy->buttons[1] != 0) {
		snLib->flipVisibility();
	}
	else if (l_button2 == false && joy->buttons[2] != 0) {
		rsLib->flipVisibility();
	}
	else if (l_button3 == false && joy->buttons[3] != 0) {
		boost::thread tmpThread(boost::bind(&BaseApplication::triggerPanoramaPTUScan, this));
	}
	else if (l_button5 == false && joy->buttons[5] != 0) {
		mPlayer->toggleFirstPersonMode();
	}
	else if (joy->buttons[7] != 0) {
		oculus->resetOrientation();
	}
	else if (l_button9 == false && joy->buttons[9] != 0) {
		globalMap->flipVisibility();
	}
	
	l_button0 = (joy->buttons[0] != 0);
	l_button1 = (joy->buttons[1] != 0);
	l_button2 = (joy->buttons[2] != 0);
	l_button3 = (joy->buttons[3] != 0);
	l_button5 = (joy->buttons[5] != 0);
	l_button9 = (joy->buttons[9] != 0);
}

void BaseApplication::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
	
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

void BaseApplication::poseCallback(const geometry_msgs::Pose::ConstPtr& pose) {
    cout<<"Pose received "<<pose<<endl;
}

void BaseApplication::topoNodesCB(const visualization_msgs::InteractiveMarkerInit::ConstPtr& data){

	for (int i=0;i<data->markers.size();i++) {
		WayPoint *wp = demoGame->getWPByName(data->markers[i].name);
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
	receivedWPs = true;	
}

void BaseApplication::closestWayPointCB(const std_msgs::String::ConstPtr& data) {
	std::string name(data->data);
	closestWP = boost::lexical_cast<int>(name.erase(0,8)); //Erase "WayPoint" before casting
}

void BaseApplication::initROS() {
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "roculus");
  hRosNode = new ros::NodeHandle();
  
  // Subscribe to the joystick input
  hRosSubJoy = new ros::Subscriber(hRosNode->subscribe<sensor_msgs::Joy>
				("/joy/visualization", 10, boost::bind(&BaseApplication::joyCallback, this, _1)));
				
  // Subscribe for the map topic (Published Once per Subscriber)
  hRosSubMap = new ros::Subscriber(hRosNode->subscribe<nav_msgs::OccupancyGrid>
				("/map", 1, boost::bind(&BaseApplication::mapCallback, this, _1)));

  //~ hRosSubPose = new ros::Subscriber(hRosNode->subscribe<geometry_msgs::Pose>
                //~ ("/robot_pose", 1, boost::bind(&BaseApplication::poseCallback, this, _1)));
                
  hRosSubNodes = new ros::Subscriber(hRosNode->subscribe<visualization_msgs::InteractiveMarkerInit>
                ("/kth_floorsix_y2_topo_markers/update_full", 1, boost::bind(&BaseApplication::topoNodesCB, this, _1)));
                
  hRosSubCloseWP = new ros::Subscriber(hRosNode->subscribe<std_msgs::String>
                ("/closest_node", 1, boost::bind(&BaseApplication::closestWayPointCB, this, _1)));


	// Subscription and binding for the 360deg images
  hRosSubRGB = new message_filters::Subscriber<sensor_msgs::CompressedImage>
				(*hRosNode, "/local_metric_map/rgb/rgb_filtered/compressed", 1);
  hRosSubDepth = new message_filters::Subscriber<sensor_msgs::CompressedImage>
				(*hRosNode, "/local_metric_map/depth/depth_filtered/compressedDepth", 1);
  rosMsgSync = new message_filters::Synchronizer<ApproximateTimePolicy>
				(ApproximateTimePolicy(15), *hRosSubDepth, *hRosSubRGB);
  rosMsgSync->registerCallback(boost::bind(&BaseApplication::syncCallback, this, _1, _2));
  
	// Subscription and binding for the 3D video and requested snapshots
  hRosSubRGBVid = new message_filters::Subscriber<sensor_msgs::CompressedImage>
				//(*hRosNode, "/head_xtion/rgb/image_raw_low_fps/compressed", 1);
				//(*hRosNode, "/head_xtion/rgb/image_color/compressed", 1);
				(*hRosNode, "/chest_xtion/rgb/image_color/reducedBW/compressed", 1);
  hRosSubDepthVid = new message_filters::Subscriber<sensor_msgs::CompressedImage>
				//(*hRosNode, "/head_xtion/depth/image_raw_low_fps/compressedDepth", 1);
				//(*hRosNode, "/head_xtion/depth_registered/image_rect/compressedDepth", 1);
				(*hRosNode, "/chest_xtion/depth_registered/image_rect/reducedBW/compressedDepth", 1);
  rosVideoSync = new message_filters::Synchronizer<ApproximateTimePolicy>
				(ApproximateTimePolicy(15), *hRosSubDepthVid, *hRosSubRGBVid);
  rosVideoSync->registerCallback(boost::bind(&BaseApplication::syncVideoCallback, this, _1, _2));
  
	// Setting up the tfListener and initialize the AsyncSpinner ROS
  //~ rosPTUClient = new Client("ptu_pan_tilt_metric_map", true);
  rosieActionClient = new actionlib::SimpleActionClient<topological_navigation::GotoNodeAction>("topological_navigation", true);
  tfListener = new tf::TransformListener(); 
  hRosSpinner = new ros::AsyncSpinner(1);
}

void BaseApplication::destroyROS() {
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

void BaseApplication::loadSavedMap() {
    cout<<"Loading saved map"<<endl;

    bool fileFound = true;
    int counter = 0;
    
    while (fileFound)
    {
        char filename_pos[50];
        sprintf(filename_pos,"snap_pos%d.txt",counter);
        ifstream pos_file(filename_pos);
        char filename_rgb[50];
        sprintf(filename_rgb,"snap_rgb%d.jpg",counter);
        char filename_depth[50];
        sprintf(filename_depth,"snap_depth%d.png",counter);

        if (pos_file)
        {
            cout<<"Loading texture "<<counter<<endl;
            Ogre::Image depthImage;
            depthImage.load(filename_depth, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

            Ogre::Image rgbImage;
            rgbImage.load(filename_rgb, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

            string vec, quat;
            getline(pos_file,vec);
            getline(pos_file,quat);

            Ogre::Vector3 vector = Ogre::StringConverter::parseVector3(vec);
            Ogre::Quaternion quaternion = Ogre::StringConverter::parseQuaternion(quat);

            rsLib->placeInScene(depthImage, rgbImage, vector, quaternion);

            counter++;
        } else {
            fileFound = false;
        }
    }

}
