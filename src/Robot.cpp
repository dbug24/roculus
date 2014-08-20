// implements a class to handle the 3D robot model

#include <Robot.h>
#include <OgreEntity.h>
#include <tf/transform_datatypes.h>

Robot::Robot(Ogre::SceneManager *mSceneMgr) {
	// set everything up for display
	robot = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	robot->setPosition(Ogre::Vector3(0.0f, 1.0f, 0.0f));
	robot->attachObject(mSceneMgr->createEntity("Rosie.mesh"));
}

Robot::~Robot() { }

void Robot::updateFrom(tf::TransformListener *tfListener) {
	using namespace Ogre;
	static tf::StampedTransform baseTF;
	static Vector3 translation = Vector3::ZERO;
	static Quaternion orientation = Quaternion::IDENTITY;
	static Quaternion qYn90 = Quaternion(Degree(90), Vector3::UNIT_Y);
	static tfScalar yaw,pitch,roll;
	static Matrix3 mRot;
	
	// get the latest robot position and orientation, transform them to Ogre and update the scene node	
	try {
		tfListener->lookupTransform("map","base_footprint",ros::Time(0), baseTF);
		
		translation.x = -baseTF.getOrigin().y();
		translation.y = baseTF.getOrigin().z();
		translation.z = -baseTF.getOrigin().x();
		translation = translation + Vector3(0.0f, 1.0f, 0.0f);
		baseTF.getBasis().getEulerYPR(yaw,pitch,roll);
		mRot.FromEulerAnglesYXZ(Radian(yaw),Radian(0.0f),Radian(0.0f));
		orientation.FromRotationMatrix(mRot);
		orientation = qYn90*orientation;
		
		robot->setPosition(translation);
		robot->setOrientation(orientation);
		
	} catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
}

Ogre::SceneNode* Robot::getSceneNode() {
	return robot;
}
