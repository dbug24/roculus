/* Extension class to handle the player motion independent from the OCULUS RIFT head motion.
Class it to be used as parent node to the Oculus handling system by Kojack.

@autor: dbug
@date: Feb.2014

 */

#ifndef _PLAYER_BODY_H_
#define _PLAYER_BODY_H_

#include <Ogre.h>
#include <OIS.h>
#include <sensor_msgs/Joy.h>
#include "Robot.h"

// Axes for LOGITECH RumblePad2
#define ROS_RJOY_X 0
#define ROS_RJOY_Y 1
#define ROS_LJOY_X 2
#define ROS_LJOY_Y 3
#define ROS_POV_X 5
#define ROS_POV_Y 6

class PlayerBody {
 private:
  Ogre::Vector3 position, velo;
  float maxSpeed, dt, fwdSpeed, udSpeed, lrSpeed;
  float turnX, turnY;
  Ogre::SceneNode* mStereoCameraParent;
  Ogre::Quaternion quad;
  bool firstPerson;

 public:
  static const int JOY_TRIGGER_LIMIT = 256; //PlusMinus
  PlayerBody(Ogre::SceneNode* mStereoCameraParent);
  ~PlayerBody();
  Ogre::Vector3 getPosition();
  float getMaxSpeed();
  void setMaxSpeed(float maxSpeed);
  void setPosition(Ogre::Vector3 position);
  void injectMouseMove(const OIS::MouseEvent& evt);
  void injectMouseDown(const OIS::MouseEvent& evt, const OIS::MouseButtonID& id);
  void injectMouseUp(const OIS::MouseEvent& evt, const OIS::MouseButtonID& id);
  void injectKeyDown(const OIS::KeyEvent& evt);
  void injectKeyUp(const OIS::KeyEvent& evt);
  void injectPOVChanged( const OIS::JoyStickEvent &arg, int pov );
  void injectAxisMoved( const OIS::JoyStickEvent &e, int axis );
  void injectSliderMoved( const OIS::JoyStickEvent &e, int sliderID );
  void injectButtonDown( const OIS::JoyStickEvent &e, int button );
  void injectButtonUp( const OIS::JoyStickEvent &e, int button );
  void injectROSJoy( const sensor_msgs::Joy::ConstPtr &joy );
  void frameRenderingQueued(const Ogre::FrameEvent& evt);
  void frameRenderingQueued(Robot *robot);
  void toggleFirstPersonMode();
  bool isFirstPerson();
};

#endif
