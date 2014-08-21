#ifndef _PLAYER_BODY_H_
#define _PLAYER_BODY_H_

#include <Ogre.h>
#include <OgreVector4.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreMath.h>
#include <OIS.h>
#include <sensor_msgs/Joy.h>
#include "Robot.h"

// Axes for LOGITECH RumblePad2
#define ROS_RJOY_X 0
#define ROS_RJOY_Y 1
#define ROS_LJOY_X 2	//2
#define ROS_LJOY_Y 3	//3
#define ROS_POV_X 4		//4
#define ROS_POV_Y 7		//5

/** \brief Handles the PlayerBody motion.
 * Extension class to handle the player motion independent from the Oculus Rift head motion.
 * The Oculus motion happens on top of this, which is achieved by making the PlayerBody's scene node the parent for the Oculus handler.
 * Since this was one of my first implementations (very unclean) this step, in fact, happens in BaseApplication.
 */
class PlayerBody {
 private:
  Ogre::Vector3 position,	/**< Position of the player.*/
				velo;		/**< Velocity of the movement.*/
  float 	maxSpeed,		/**< Maximum velocity.*/
			dt,				/**< Difference in time between movements.*/
			fwdSpeed,		/**< Speed in forward/backward direction.*/
			udSpeed,		/**< Speed for up/down movements.*/
			lrSpeed;		/**< Speed for left/right movements.*/
  float		turnX,			/**< Angular velocity for rolling. (Not implemented right now. Can probably be removed).*/
			turnY;			/**< Angular velocity for yawing.*/
  Ogre::SceneNode* mStereoCameraParent;	/**< The scene node for the plaber body movement. Camera/Oculus Rift movement is applied on top of this.*/
  Ogre::Quaternion quad;	/**< The player orientation.*/
  bool firstPerson;			/**< True if in first-person mode.*/
  int offset;				/**< Offset for the head orientation. =0:none, >0: right-turn, <0: left turn.*/

 public:
  static const int JOY_TRIGGER_LIMIT = 256; 
  /**< Minimal trigger signal to accept inputs. (Cancelling noise).*/
  PlayerBody(Ogre::SceneNode* mStereoCameraParent);
  /**< Constructor. Sets up the PlayerBody object, based on the given scene node. (Could be changed to SceneMgr and create the node itself...)*/
  ~PlayerBody();
  /**< Default destructor.*/
  Ogre::Vector3 getPosition();
  /**< Returns the current player position.*/
  float getMaxSpeed();
  /**< Returns the maximum speed.*/
  void setMaxSpeed(float maxSpeed);
  /**< Sets the maximum speed.*/
  void setPosition(Ogre::Vector3 position);
  /**< Sets the PlayerPosition. Usually not used -> standard is event injection.*/
  void injectMouseMove(const OIS::MouseEvent& evt);
  /**< OIS: Process a mouse movement, forwarded from the BaseApplication.*/
  void injectMouseDown(const OIS::MouseEvent& evt, const OIS::MouseButtonID& id);
  /**< OIS: Process a mouse button event, forwarded from the BaseApplication.*/
  void injectMouseUp(const OIS::MouseEvent& evt, const OIS::MouseButtonID& id);
  /**< OIS: Process a mouse button event, forwarded from the BaseApplication.*/
  void injectKeyDown(const OIS::KeyEvent& evt);
  /**< OIS: Process a keyboard event.*/
  void injectKeyUp(const OIS::KeyEvent& evt);
  /**< OIS: Process a keyboard event.*/
  void injectPOVChanged( const OIS::JoyStickEvent &arg, int pov );
  /**< OIS: Process a POV-joystick event.*/
  void injectAxisMoved( const OIS::JoyStickEvent &e, int axis );
  /**< OIS: Process the movement of an axis.*/
  void injectSliderMoved( const OIS::JoyStickEvent &e, int sliderID );
  /**< OIS: Process the movement of a slider.*/
  void injectButtonDown( const OIS::JoyStickEvent &e, int button );
  /**< OIS: Process a button event on the joystick.*/
  void injectButtonUp( const OIS::JoyStickEvent &e, int button );
  /**< OIS: Process a button event on the joystick.*/
  void injectROSJoy( const sensor_msgs::Joy::ConstPtr &joy );
  /**< ROS: Process an incomming joystick message from ROS. This combines the various events from the OIS system by processing the entire joystick state.*/
  void frameRenderingQueued(const Ogre::FrameEvent& evt);
  /**< Given a Ogre::FrameEvent, apply all commands that result from the various inputs.*/
  void frameRenderingQueued(Robot *robot);
  /**< Given a Robot, update the player position to the robot's posititon. This effectively is the first-person mode.*/
  void toggleFirstPersonMode();
  /**< Mark the PlayerBody to run in first-person or free-view mode. Note that this will just set the boolean flag - the behaviour change is achieved by calling different frameRenderingQueued methods.*/
  bool isFirstPerson();
  /**< Returns if the PlayerBody shall be mounted to a Robot or not.*/
};

#endif
