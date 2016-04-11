/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ESTIMATOR_NODE_H
#define ESTIMATOR_NODE_H

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

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>


using namespace Eigen;

class EstimatorNode {
 public:
  EstimatorNode();
  ~EstimatorNode();

  void Publish();

 private:

  // Define here the matrices and vectors of the Kalman filter

  bool hasChanged = false;

  double sigma_nx;
  double sigma_nz;
  double sigma_nu;

  double dT;
  double time;
  double precTick;
  double ticks;

  // subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber imu_sub_;

  ros::Publisher pose_pub;
  geometry_msgs::PoseStamped msgPose_;

  ros::Timer timer_;

  void PoseCallback(
      const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void ImuCallback(
      const sensor_msgs::ImuConstPtr& imu_msg);

  void TimedCallback(
      const ros::TimerEvent& e);

  void updateMatrixWithDelta();

  void getTime();
};


#endif // ESTIMATOR_NODE_H
