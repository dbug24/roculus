#include "PlayerBody.h"

PlayerBody::PlayerBody(Ogre::SceneNode* mStereoCameraParent)
  : position(Ogre::Vector3::ZERO),
    velo(Ogre::Vector3::UNIT_SCALE*1.0),
    maxSpeed(5.0f),
    fwdSpeed(0.0f),
    lrSpeed(0.0f),
    udSpeed(0.0f),
    dt(0.0f),
    turnX(0.0f),
    turnY(0.0f),
    firstPerson(false),
    quad(Ogre::Quaternion::IDENTITY),
    offset(0)
{
  this->mStereoCameraParent = mStereoCameraParent;
}

PlayerBody::~PlayerBody() {
  //Camera node should be handled by the root system
}

Ogre::Vector3 PlayerBody::getPosition() {
  return position;
}

float PlayerBody::getMaxSpeed() {
  return maxSpeed;
}

void PlayerBody::setMaxSpeed(float maxSpeed) {
  this->maxSpeed = maxSpeed;
}

void PlayerBody::setPosition(Ogre::Vector3 position) {
  this->position = position;
}

void PlayerBody::injectMouseMove(const OIS::MouseEvent& evt) {
  //turn orientation (yaw, pitch);
}

void PlayerBody::injectMouseDown(const OIS::MouseEvent& evt, const OIS::MouseButtonID& id) {
  //nothing
}

void PlayerBody::injectMouseUp(const OIS::MouseEvent& evt, const OIS::MouseButtonID& id) {
  //nothing
}

void PlayerBody::injectKeyDown(const OIS::KeyEvent& evt) {
	// From the event, mark what actions should be taken during the rendering
  if (evt.key == OIS::KC_W || evt.key == OIS::KC_UP) { fwdSpeed = -1.0f; }
  else if (evt.key == OIS::KC_S || evt.key == OIS::KC_DOWN) { fwdSpeed = 1.0f; }
  else if (evt.key == OIS::KC_A || evt.key == OIS::KC_LEFT) { lrSpeed = -1.0f; }
  else if (evt.key == OIS::KC_D || evt.key == OIS::KC_RIGHT) { lrSpeed = 1.0f; }
  else if (evt.key == OIS::KC_PGUP) { udSpeed = 1.0f; }
  else if (evt.key == OIS::KC_PGDOWN) { udSpeed = -1.0f; }
  else if (evt.key == OIS::KC_Q) { turnY = -1.5f; }
  else if (evt.key == OIS::KC_E) { turnY = 1.5f;}
}

void PlayerBody::injectKeyUp(const OIS::KeyEvent& evt) {
	// From the event, mark what actions should be taken during the rendering
  if (evt.key == OIS::KC_W || evt.key == OIS::KC_UP || evt.key == OIS::KC_S || evt.key == OIS::KC_DOWN) 
	{ fwdSpeed = 0.0f; }
  else if (evt.key == OIS::KC_A || evt.key == OIS::KC_LEFT || evt.key == OIS::KC_D || evt.key == OIS::KC_RIGHT)
	{ lrSpeed = 0.0f; }
  else if (evt.key == OIS::KC_PGUP || evt.key == OIS::KC_PGDOWN) { udSpeed = 0.0f; }
  else if (evt.key == OIS::KC_Q || evt.key == OIS::KC_E) { turnY = 0.0f; }
}

void PlayerBody::injectAxisMoved( const OIS::JoyStickEvent &evt, int axis ) {
	// From the event, mark what actions should be taken during the rendering
  switch (axis) {
  case 0: if (evt.state.mAxes[0].abs > JOY_TRIGGER_LIMIT) { lrSpeed = 1.0f; }
    else if (evt.state.mAxes[0].abs < -JOY_TRIGGER_LIMIT) { lrSpeed = -1.0f; }
    else { lrSpeed = 0.0f; }
    break;
  case 1: if (evt.state.mAxes[1].abs > JOY_TRIGGER_LIMIT) { udSpeed = -1.0f; }
    else if (evt.state.mAxes[1].abs < -JOY_TRIGGER_LIMIT) { udSpeed = 1.0f; }
    else { udSpeed = 0.0f; }
    break;
  case 3: if (evt.state.mAxes[3].abs > JOY_TRIGGER_LIMIT || evt.state.mAxes[3].abs < -JOY_TRIGGER_LIMIT) { turnY = ((float)evt.state.mAxes[3].abs)/20000.0f; }
    else { turnY = 0.0f; }
    break;
  case 4: if (evt.state.mAxes[4].abs > JOY_TRIGGER_LIMIT || evt.state.mAxes[4].abs < -JOY_TRIGGER_LIMIT) { fwdSpeed = ((float)evt.state.mAxes[4].abs)/30000.0f; } else { fwdSpeed = 0.0f; }
    break;
  default: //nothing, other axes are unused
    break;
  }
}

void PlayerBody::injectPOVChanged( const OIS::JoyStickEvent &arg, int pov ) {
	// From the event, mark what actions should be taken during the rendering
	if (arg.state.mPOV[pov].direction == OIS::Pov::Centered) { udSpeed = lrSpeed = 0.0f; }
	else if (arg.state.mPOV[pov].direction & OIS::Pov::North) { udSpeed = 1.0f; }
	else if (arg.state.mPOV[pov].direction & OIS::Pov::South) { udSpeed = -1.0f; }
	else if (arg.state.mPOV[pov].direction & OIS::Pov::East) { lrSpeed = -1.0f; }
	else if (arg.state.mPOV[pov].direction & OIS::Pov::West) { lrSpeed = 1.0f; }
}

void PlayerBody::injectSliderMoved( const OIS::JoyStickEvent &evt, int sliderID ) {
  //nothing
}

void PlayerBody::injectButtonDown( const OIS::JoyStickEvent &evt, int button ) {
  //nothing, evtl, one-time events, screenshot, POIs
}

void PlayerBody::injectButtonUp( const OIS::JoyStickEvent &evt, int button ) {
  //nothing
}

void PlayerBody::frameRenderingQueued(const Ogre::FrameEvent& evt) {
	// Apply all steps that were recorded with the injection methods:
	
	//acceleration:
	dt = ((float) evt.timeSinceLastFrame); 
	quad = mStereoCameraParent->getOrientation();
	velo.z = maxSpeed*fwdSpeed;
	velo.y = maxSpeed*udSpeed;
	velo.x = maxSpeed*lrSpeed;
	//camera translation:
	mStereoCameraParent->translate(Ogre::Vector3::UNIT_X*velo.x*dt, Ogre::Node::TS_LOCAL);
	mStereoCameraParent->translate(Ogre::Vector3::UNIT_Y*velo.y*dt, Ogre::Node::TS_WORLD);
	mStereoCameraParent->translate(quad.zAxis()*velo.z*dt);

	//set camera node orientation (Player Body can just be yawed).
	mStereoCameraParent->yaw(Ogre::Radian(Ogre::Real(-turnY*dt)));
}

void PlayerBody::frameRenderingQueued(Robot *robot) {
	// Place the player on the robot's position.
	// The quaternion is needed due to the coordinates of the robot avatar mesh
	// (The mesh coordinates could be corrected with blender to match the Ogre coords, in order to avoid the transforms)
	// (... this was done for all following models, like the game objects)
	static Ogre::Quaternion qRot(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y);
	static Ogre::Quaternion lOffset(Ogre::Degree(-160), Ogre::Vector3::UNIT_Y);
	static Ogre::Quaternion rOffset(Ogre::Degree(160), Ogre::Vector3::UNIT_Y);
	
	mStereoCameraParent->setPosition(Ogre::Vector3::UNIT_Y*0.7+robot->getSceneNode()->getPosition());
	if (offset > 0) {
		mStereoCameraParent->setOrientation(rOffset*qRot*robot->getSceneNode()->getOrientation());
	} else if (offset < 0) {
		mStereoCameraParent->setOrientation(lOffset*qRot*robot->getSceneNode()->getOrientation());
	} else {
		mStereoCameraParent->setOrientation(qRot*robot->getSceneNode()->getOrientation());
	}
}

void PlayerBody::injectROSJoy( const sensor_msgs::Joy::ConstPtr &joy ) {
	// wow, this is so much easier with ROS...
	lrSpeed = -joy->axes[ROS_LJOY_X];
	udSpeed = joy->axes[ROS_LJOY_Y];
	fwdSpeed = -joy->axes[ROS_RJOY_Y];
	turnY = -joy->axes[ROS_RJOY_X]*2.0f;
	offset = joy->axes[ROS_POV_X];
}

void PlayerBody::toggleFirstPersonMode() {
	firstPerson = !firstPerson;
}

bool PlayerBody::isFirstPerson() {
	return firstPerson;
}
