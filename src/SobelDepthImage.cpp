#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <float.h>
#include <limits.h>

using namespace sensor_msgs;

#define OPENCV_WINDOW "Sobel Window"

class SobelDepthImage {
public:
	SobelDepthImage(int argc, char **argv);
	~SobelDepthImage();
	int spin();
protected:
	void sobelCallback( const ImageConstPtr& );
	
	ros::NodeHandle nh;
	ros::Rate loopRate;
	
	image_transport::ImageTransport it;
	image_transport::Publisher out;
	image_transport::Subscriber in;
};

SobelDepthImage::SobelDepthImage(int argc, char **argv) : it(nh), loopRate(20) {	
	const std::string topic_in(argv[1]);
	const std::string topic_out(topic_in+"/sobel");
	
	out = it.advertise(topic_out, 3);
	in =  it.subscribe(topic_in, uint32_t(10), &SobelDepthImage::sobelCallback, this);
	
	cv::namedWindow(OPENCV_WINDOW);
}

SobelDepthImage::~SobelDepthImage() {
	cv::destroyWindow(OPENCV_WINDOW);
}

void SobelDepthImage::sobelCallback( const ImageConstPtr& image ) {
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
		using namespace cv;
		Mat gauss, gradX, gradY, grad, thres;
		GaussianBlur(cv_ptr->image, gauss, Size(5,5), 0, 0, BORDER_DEFAULT);
		Sobel(gauss, gradX, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT);
		Sobel(gauss, gradY, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT);
		gradX = abs(gradX);
		gradY = abs(gradY);
		bitwise_or(gradX, gradY, grad);
		grad.convertTo(thres, CV_32FC1);
		double dynThres = 0.25*mean(grad).val[0];
		std::cout << dynThres << std::endl;
		threshold(thres, cv_ptr->image, dynThres , 1.0 , 0);
		
		//~ imshow(OPENCV_WINDOW, cv_ptr->image);
		//~ waitKey(3);
		
		out.publish(cv_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Sobel_Depth_Image");
	SobelDepthImage sdi(argc, argv);
	ros::spin();
	return 0;
}
