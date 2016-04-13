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

#include "estimator.h"


//Init KALMAN FILTER variables
// state vectors
Eigen::VectorXd x_before(6);
Eigen::VectorXd x_predicted(6);
Eigen::VectorXd x_t(6);

// input Vector
Eigen::Vector3d u;

//Covariance of the state
Eigen::MatrixXd P(6,6);
Eigen::MatrixXd P_predicted(6,6);
Eigen::MatrixXd P_before(6,6);

//Covariance of Meas
Eigen::Matrix3d R;

//Covariance of system
Eigen::MatrixXd Q(6,6);

//z_t = H*x_t + n_z
Eigen::MatrixXd H(3,6);        //(3,6)
Eigen::MatrixXd H_T(6,3);      //(6,3)

//Measurements vectors
Eigen::Vector3d z_t(3);
Eigen::Vector3d z_predicted(3);

//x_t = F*x_before + G*u
Eigen::MatrixXd F(6,6);
Eigen::MatrixXd G(6,3);        //(6,3)


Eigen::MatrixXd K(6,6);

//IMU bias
Eigen::Vector3d bias;
double count = 0;


EstimatorNode::EstimatorNode() {


  ros::NodeHandle nh("~");

  pose_sub_ = nh.subscribe("/firefly/fake_gps/pose",  1, &EstimatorNode::PoseCallback, this);
  imu_sub_  = nh.subscribe("/firefly/imu",            1, &EstimatorNode::ImuCallback, this);

  nh.getParam("sigma_nx", sigma_nx);
  nh.getParam("sigma_nz", sigma_nz);
  nh.getParam("sigma_nu", sigma_nu);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/pose", 1);

  timer_ = nh.createTimer(ros::Duration(0.1), &EstimatorNode::TimedCallback, this);

  bias[0] = 0.0;
  bias[1] = 0.0;
  bias[2] = 0.0;

  dT = 0;
  time = 0;
  precTick = 0;
  ticks = 0;

  x_t(0) = 0;
  x_t(1) = 0;
  x_t(2) = 0;
  x_t(3) = 0;
  x_t(4) = 0;
  x_t(5) = 0;

  P << sigma_nx, 0, 0, 0, 0, 0,
       0, sigma_nx, 0, 0, 0, 0,
       0, 0, sigma_nx, 0, 0, 0,
       0, 0, 0, sigma_nx, 0, 0,
       0, 0, 0, 0, sigma_nx, 0,
       0, 0, 0, 0, 0, sigma_nx;

  H << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0;

  H_T = H.transpose();

  R << sigma_nz, 0, 0,
       0, sigma_nz, 0,
       0, 0, sigma_nz;

  updateMatrixWithDelta();

}

EstimatorNode::~EstimatorNode() { }

void EstimatorNode::Publish()
{
  //publish your data
  ROS_INFO("Publishing ...");

  pose_pub.publish(msgPose_);
}

void EstimatorNode::PoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {

  if(count == 300){

      ROS_INFO_ONCE("Estimator got first POSE message.");
      msgPose_.header.stamp = pose_msg->header.stamp;
      msgPose_.header.seq = pose_msg->header.seq;
      msgPose_.header.frame_id =pose_msg->header.frame_id;

      //Data adquisition
      z_t(0) = pose_msg ->pose.position.x;
      z_t(1) = pose_msg ->pose.position.y;
      z_t(2) = pose_msg ->pose.position.z;


      //CORRECTION
      K = P * H_T * ((H * P * H_T) + R).inverse();

      x_t = x_predicted + K * (z_t-z_predicted);
      P = P_predicted - K * H * P_predicted;

      //update Pose predicted
      msgPose_.pose.position.x = x_t[0];
      msgPose_.pose.position.y = x_t[1];
      msgPose_.pose.position.z = x_t[2];

     // Publish();
  }
}

void EstimatorNode::ImuCallback(
    const sensor_msgs::ImuConstPtr& imu_msg) {

  ROS_INFO_ONCE("Estimator got first IMU message.");

  //Data adquisition
  u(0) = imu_msg ->linear_acceleration.x;
  u(1) = imu_msg ->linear_acceleration.y;
  u(2) = imu_msg ->linear_acceleration.z;


  if( count <= 299){
      count++;

      //get BIAS
      if(count == 300){
          bias[0] = bias[0]/count;
          bias[1] = bias[1]/count;
          bias[2] = bias[2]/count;
      }else{
          bias[0] = bias[0] + u(0);
          bias[1] = bias[1] + u(1);
          bias[2] = bias[2] + u(2);
      }

      ROS_INFO_STREAM ("Count: " << sigma_nu );

  }else if (count == 300){


      ROS_INFO_STREAM ("BIAS: " << bias );
      msgPose_.header.stamp = imu_msg->header.stamp;
      msgPose_.header.seq = imu_msg->header.seq;
      msgPose_.header.frame_id =imu_msg->header.frame_id;

      ROS_INFO_STREAM ("U: " << u);

      u(0) = u(0) - bias[0];
      u(1) = u(1) - bias[1];
      u(2) = u(2) - bias[2];

      //update matrix F, G, Q
      updateMatrixWithDelta();

      //PREDICTION
      x_predicted = F * x_t + G * u;
      P_predicted = F * P * F.transpose() +  Q;
      z_predicted = H * x_predicted;

      x_t = x_predicted;
      P = P_predicted;

      //update Pose predicted
      msgPose_.pose.position.x = z_predicted[0];
      msgPose_.pose.position.y = z_predicted[1];
      msgPose_.pose.position.z = z_predicted[2];

     // Publish();
  }
}

void EstimatorNode::TimedCallback(
      const ros::TimerEvent& e){
   ROS_INFO_ONCE("Timer initiated.");
   Publish();
}


void EstimatorNode::updateMatrixWithDelta(){

    precTick = ticks;

    ticks = (double) cv::getTickCount();
    dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

    time += dT;

    double dt_2 = pow(dT,2);
    double dt_3 = pow(dT,3);

    Q << 1/2*dt_3, 0, 0, 1/2*dt_2, 0, 0,
         0, 1/2*dt_3, 0, 0, 1/2*dt_2, 0,
         0, 0, 1/2*dt_3, 0, 0, 1/2*dt_2,
         1/2*dt_2, 0, 0, 1/2*dt_2, 0, 0,
         0, 1/2*dt_2, 0, 0, 1/2*dt_2, 0,
         0, 0, 1/2*dt_2, 0, 0, 1/2*dt_2;

    Q = sigma_nu * Q;

    F << 1, 0, 0, dT, 0, 0,
         0, 1, 0, 0, dT, 0,
         0, 0, 1, 0, 0, dT,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    G << 1/2*dt_2, 0, 0,
         0, 1/2*dt_2, 0,
         0, 0, 1/2*dt_2,
         dT, 0, 0,
         0, dT, 0,
         0, 0, dT;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "estimator");

  EstimatorNode estimator_node;

  ros::spin();

  return 0;
}
