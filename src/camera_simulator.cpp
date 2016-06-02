/*
 * camera_simulator.cpp
 *
 *      Author:
 *         Nicolas Acero
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

sensor_msgs::Image currentFrame;

void readImg(const std::string imgName) {
  cv::Mat img = cv::imread("/home/n/caltech-lanes/highway45/" + imgName);
  cv_bridge::CvImage img_bridge;

  std_msgs::Header header; // empty header
  header.stamp = ros::Time::now(); // time
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
  currentFrame = *img_bridge.toImageMsg(); // from cv_bridge to sensor_msgs::Image
  if(currentFrame.step == 0) {
    std::cout << "Error: No image with name " << imgName << " received" << std::endl;
  }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "camera_simulator");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);


		/**
	   * The advertise() function is how you tell ROS that you want to
	   * publish on a given topic name. This invokes a call to the ROS
	   * master node, which keeps a registry of who is publishing and who
	   * is subscribing. After this advertise() call is made, the master
	   * node will notify anyone who is trying to subscribe to this topic name,
	   * and they will in turn negotiate a peer-to-peer connection with this
	   * node.  advertise() returns a Publisher object which allows you to
	   * publish messages on that topic through a call to publish().  Once
	   * all copies of the returned Publisher object are destroyed, the topic
	   * will be automatically unadvertised.
	   *
	   * The second parameter to advertise() is the size of the message queue
	   * used for publishing messages.  If messages are published more quickly
	   * than we can send them, the number here specifies how many messages to
	   * buffer up before throwing some away.
	   */

     image_transport::Publisher image_pub = it.advertise("/camera/rgb/image_rect_mono", 1);


		//This node needs to publish at least at a rate of 20hz or navigational stack will complain
    ros::Rate loop_rate(30);
    std::stringstream ss;
    int i = 0;
    while(ros::ok()) {
      i++;
      if(i < 10) {
        ss << "IMG_000" << i << ".png";
      }
      else if(i < 100) {
        ss << "IMG_00" << i << ".png";
      }
      else if(i < 1000) {
        ss << "IMG_0" << i << ".png";
      }

      else if(i < 1152) {
        ss << "IMG_" << i << ".png";
      }

      else {
        i = 1;
        ss << "IMG_0001.png";
      }

      readImg(ss.str());
      ss.str(std::string());
      image_pub.publish(currentFrame);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
