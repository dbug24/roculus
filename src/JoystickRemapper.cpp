#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
using namespace std;
using namespace ros;
using namespace sensor_msgs;

class JoystickRemapper {
public:
	JoystickRemapper(int, int, char**);
	~JoystickRemapper();
	int spin();
protected:
	int MODE_SELECT; //index of the joystick button
	void joyCallback( const Joy::ConstPtr& );
	
	NodeHandle *nh;
	Rate *loopRate;
	
	Publisher *joyROBOT;
	Publisher *joyVISUAL;
	Subscriber *joyORIGINAL;
};

JoystickRemapper::JoystickRemapper( int mode_button_idx, int argc, char** argv ) {
	MODE_SELECT = mode_button_idx;
	
	nh = NULL;
	loopRate = NULL;
	joyROBOT = joyVISUAL = NULL;
	joyORIGINAL = NULL;
	ros::init(argc, argv, "JoystickRemapper");
	nh = new NodeHandle();
	loopRate = new Rate(20);
	
	joyROBOT = new Publisher(nh->advertise<Joy>("joy", 30));
	joyVISUAL = new Publisher(nh->advertise<Joy>("joy/visualization", 30));
	joyORIGINAL = new Subscriber(nh->subscribe<Joy>("joy/original", 10, boost::bind(&JoystickRemapper::joyCallback, this, _1)));
}

JoystickRemapper::~JoystickRemapper() {
	ros::shutdown();
	if (loopRate) {
		delete loopRate;
		loopRate = NULL;
	}
	if (nh) {
		delete nh;
		nh = NULL;
	}
	if (joyROBOT) {
		delete joyROBOT;
		joyROBOT = NULL;
	}
	if (joyVISUAL) {
		delete joyVISUAL;
		joyVISUAL = NULL;
	}
	if (joyORIGINAL) {
		delete joyORIGINAL;
		joyORIGINAL = NULL;
	}
}

int JoystickRemapper::spin() {
	while (ros::ok()) {
		ros::spinOnce();
		loopRate->sleep();
	}
	return 0;
}

void JoystickRemapper::joyCallback( const Joy::ConstPtr &joy ) {
	if (joy->buttons[MODE_SELECT] == 1) {
		//~ joyROBOT->publish(joy);
	} else {
		joyVISUAL->publish(joy);
	}
}

int main(int argc, char **argv) {
	JoystickRemapper jr(4, argc, argv);
	return jr.spin();
}
