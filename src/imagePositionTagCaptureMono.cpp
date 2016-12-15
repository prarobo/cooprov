#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose.h"
#include "ar_track_alvar/AlvarMarkers.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
//#include <opencv/cxcore.hpp>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

/**
 * Saves time tagged images
 */

char imageName[] = "image_";
char imageFoldername[] = "/home/prasanna/ros_hydro_arv/src/openrov/image_position_tag_data";
static const char WINDOW[] = "Image window";
char *filename = (char*) calloc(256, sizeof(char));
char posFilename[] = "pos_file.txt";

ofstream posFile;

int imageCounter = 1;
double pauseTime = 1/5; //seconds

cv_bridge::CvImagePtr camImage;
ros::Time currTagTime, prevTagTime;

void tagCallback(const ar_track_alvar::AlvarMarkers& msg) {
  currTagTime = ros::Time::now();
  double timeDiff = currTagTime.toSec() - prevTagTime.toSec();

  if (timeDiff > pauseTime && !msg.markers.empty()){
	  prevTagTime = currTagTime;
	  ROS_INFO("Tag captured time difference (in sec) %f", timeDiff);

	  sprintf(filename, "%s/%s%05d.png",imageFoldername, imageName, imageCounter);
	  imwrite( filename, camImage->image );

	  geometry_msgs::Pose currPose = msg.markers[0].pose.pose;
	  posFile<<"Image"<<"\t"<<imageCounter<<"\t";
	  posFile<<currPose.position.x<<"\t";
	  posFile<<currPose.position.y<<"\t";
	  posFile<<currPose.position.z<<"\t";
	  posFile<<currPose.orientation.x<<"\t";
	  posFile<<currPose.orientation.y<<"\t";
	  posFile<<currPose.orientation.z<<"\t";
	  posFile<<currPose.orientation.w<<"\t";
	  posFile<<"\n";

	  imageCounter++;
  }
}

void imageCallback(const sensor_msgs::Image::ConstPtr& camImageMsg)	{
  //cout<<"Left image received ..."<<endl;

  try  {
    camImage = cv_bridge::toCvCopy(camImageMsg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //namedWindow(LEFT_WINDOW);
  //imshow(LEFT_WINDOW, leftCamImage->image);
  //waitKey(3);

  //if(leftCamImage)	leftCamImage->image.release();
}

int main(int argc, char **argv) {

	char *tempStr = (char*) calloc(256, sizeof(char));
	sprintf(tempStr, "rm -r %s", imageFoldername);
	system(tempStr);

	sprintf(tempStr, "mkdir %s", imageFoldername);
	system(tempStr);

	sprintf(tempStr, "%s/%s", imageFoldername, posFilename);
	posFile.open(tempStr, ios::out | ios::trunc);

	ros::init(argc, argv, "image_position_tagger");
	ros::NodeHandle n;

	prevTagTime = ros::Time::now();

	ros::Subscriber arSub = n.subscribe("ar_pose_marker", 10, tagCallback);
	ros::Subscriber imageSub = n.subscribe("mycam/image_raw", 10, imageCallback);

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	posFile.close();
	free(tempStr);
	return 0;
}
