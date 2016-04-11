#ifndef GETPATH_H
#define GETPATH_H

#include <boost/bind.hpp>
//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <stdio.h>

#include <ros/ros.h>


//OpenCV
#include <opencv/cv.h>

//#include <ros/TimerEvent.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


using namespace Eigen;

class GetPathNode {
 public:
  GetPathNode();
  ~GetPathNode();

  void Publish();

 private:


  // subscribers
  ros::Subscriber KF_sub_;
  ros::Subscriber GPS_sub_;
  ros::Subscriber GT_sub_;

  ros::Publisher KFPath_pub_;
  ros::Publisher GPSPath_pub_;
  ros::Publisher GTPath_pub_;

  nav_msgs::Path msgKFPath_;
  nav_msgs::Path msgGPSPath_;
  nav_msgs::Path msgGTPath_;


  geometry_msgs::PoseStamped msgPose_;

  ros::Timer timer_;

  void poseKFCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void fakeGPSCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void groundTruthCallback( const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void TimedCallback(const ros::TimerEvent& e);
};

#endif // GETPATH_H
