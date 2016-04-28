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
#include <tf_conversions/tf_eigen.h>


//Init KALMAN FILTER variables
// state vectors
Eigen::VectorXd x_apriori(6);
Eigen::VectorXd x_post(6);
Eigen::VectorXd x_hat(6);


// input Vector
Eigen::VectorXd u(6);

//Covariance of the state
Eigen::MatrixXd P_post(6,6);
Eigen::MatrixXd P_apriori(6,6);
Eigen::MatrixXd P_hat(6,6);

//Covariance of Meas
Eigen::Matrix3d R;

//Covariance of system
Eigen::MatrixXd Q(6,6);


Eigen::MatrixXd I(6,6);

//z_t = H*x_t + n_z
Eigen::MatrixXd H(3,6);
Eigen::MatrixXd H_T(6,3);

//Measurements vectors
Eigen::Vector3d z_t(3);
Eigen::Vector3d z_predicted(3);

//x_t = F*x_before + G*u
Eigen::MatrixXd F(6,6);
Eigen::MatrixXd G(6,3);

Eigen::MatrixXd K(6,6);

//IMU bias
Eigen::Vector3d accBias;
double count = 0;

Eigen::Vector3d accXYZ;


EstimatorNode::EstimatorNode() {


  ros::NodeHandle nh("~");

  pose_sub_ = nh.subscribe("/firefly/fake_gps/pose",        1, &EstimatorNode::PoseCallback, this);
  imu_sub_  = nh.subscribe("/firefly/imu",                  1, &EstimatorNode::ImuCallback, this);
  gt_sub_  = nh.subscribe("/firefly/ground_truth/pose",    1, &EstimatorNode::GroundTruthCallback, this);

  nh.getParam("sigma_nx", sigma_nx);
  nh.getParam("sigma_nz", sigma_nz);
  nh.getParam("sigma_nu", sigma_nu);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/pose", 1);

  timer_ = nh.createTimer(ros::Duration(0.1), &EstimatorNode::TimedCallback, this);

  accBias[0] = 0.0;
  accBias[1] = 0.0;
  accBias[2] = 0.0;

  accXYZ[0] = 0.0;
  accXYZ[1] = 0.0;
  accXYZ[2] = 0.0;

  dT = 0;
  lastTime = 0;

  x_hat(0) = 0;
  x_hat(1) = 0;
  x_hat(2) = 0;
  x_hat(3) = 0;
  x_hat(4) = 0;
  x_hat(5) = 0;

  P_hat << sigma_nx, 0, 0, 0, 0, 0,
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

  I << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;

  updateMatrixWithDelta(0.0);

}

EstimatorNode::~EstimatorNode() { }

void EstimatorNode::Publish()
{
  //publish your data
  //ROS_INFO("Publishing ...");

  pose_pub.publish(msgPose_);
}

void EstimatorNode::PoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {

  if(count == 300){

      ROS_INFO_ONCE("Estimator got first POSE message.");
      msgPose_.header.stamp = pose_msg->header.stamp;
      msgPose_.header.seq = pose_msg->header.seq;
      msgPose_.header.frame_id ="world";

      //Data adquisition
      z_t(0) = pose_msg ->pose.position.x;
      z_t(1) = pose_msg ->pose.position.y;
      z_t(2) = pose_msg ->pose.position.z;


      //CORRECTION
      K = P_hat * H_T * ((H * P_hat * H_T) + R).inverse();
      x_post = x_hat + K * (z_t-z_predicted);

      Eigen::MatrixXd TEMP = (I - (K * H));
      P_post = TEMP * P_hat * TEMP.transpose() + K * R * K.transpose();

      x_hat = x_post;
      P_hat = P_post;

      //update Pose predicted
      msgPose_.pose.position.x = x_hat[0];
      msgPose_.pose.position.y = x_hat[1];
      msgPose_.pose.position.z = x_hat[2];

      msgPose_.pose.orientation.x = quaternion(0);
      msgPose_.pose.orientation.y = quaternion(1);
      msgPose_.pose.orientation.z = quaternion(2);
      msgPose_.pose.orientation.w = quaternion(3);

  }
}

void EstimatorNode::ImuCallback(
    const sensor_msgs::ImuConstPtr& imu_msg) {

  ROS_INFO_ONCE("Estimator got first IMU message.");

  //Data adquisition
  u(0) = imu_msg -> linear_acceleration.x;
  u(1) = imu_msg -> linear_acceleration.y;
  u(2) = imu_msg -> linear_acceleration.z;

  u(3) = imu_msg -> angular_velocity.x;
  u(4) = imu_msg -> angular_velocity.y;
  u(5) = imu_msg -> angular_velocity.z;

  if( count <= 299){
      count++;
      //get BIAS
      //ROS_INFO_STREAM("Count " << count);
      if(count == 300){
          accBias[0] = accBias[0]/count;
          accBias[1] = accBias[1]/count;
          accBias[2] = (accBias[2]/count);
          ROS_INFO_STREAM("BIAS: " << accBias);
      }else{
          accBias[0] = accBias[0] + u(0);
          accBias[1] = accBias[1] + u(1);
          accBias[2] = accBias[2] + (u(2) - GRAVETAT);
      }

  }else if (count == 300){

          msgPose_.header.stamp = imu_msg->header.stamp;
          msgPose_.header.seq = imu_msg->header.seq;
          msgPose_.header.frame_id = "world";

          //update matrix F, G, Q
          updateMatrixWithDelta(msgPose_.header.stamp.toSec());

          //ROTATE accelerations
          accXYZ = rotateAcc();

          //PREDICTION
          x_apriori = F * x_hat + G * accXYZ;
          P_apriori = F * P_hat * F.transpose() +  Q;
          z_predicted = H * x_apriori;

          x_hat = x_apriori;
          P_hat = P_apriori;

          //update Pose predicted
          msgPose_.pose.position.x = x_hat[0];
          msgPose_.pose.position.y = x_hat[1];
          msgPose_.pose.position.z = x_hat[2];

          msgPose_.pose.orientation.x = quaternion(0);
          msgPose_.pose.orientation.y = quaternion(1);
          msgPose_.pose.orientation.z = quaternion(2);
          msgPose_.pose.orientation.w = quaternion(3);


         // ROS_INFO_STREAM("X_HAT" << x_hat );

  }
}

void EstimatorNode::GroundTruthCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg_1){

    quaternion(0) = pose_msg_1 ->pose.orientation.x;
    quaternion(1) = pose_msg_1 ->pose.orientation.y;
    quaternion(2) = pose_msg_1 ->pose.orientation.z;
    quaternion(3) = pose_msg_1 ->pose.orientation.w;
}

void EstimatorNode::TimedCallback(
      const ros::TimerEvent& e){
   ROS_INFO_ONCE("Timer initiated.");
   Publish();
}


void EstimatorNode::updateMatrixWithDelta(double time){

    dT = time - lastTime;
   // dT = dT*1E-9;
    lastTime = time;

    dt_2 = 0.5*dT*dT;
    dt_3 = 0.5*dT*dT*dT;

    Q << dt_3, 0, 0, dt_2, 0, 0,
         0, dt_3, 0, 0, dt_2, 0,
         0, 0, dt_3, 0, 0, dt_2,
         dt_2, 0, 0, dt_2, 0, 0,
         0, dt_2, 0, 0, dt_2, 0,
         0, 0, dt_2, 0, 0, dt_2;

    Q = (sigma_nu * Q);

    F << 1, 0, 0, dT, 0, 0,
         0, 1, 0, 0, dT, 0,
         0, 0, 1, 0, 0, dT,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    G << dt_2, 0, 0,
         0, dt_2, 0,
         0, 0, dt_2,
         dT, 0, 0,
         0, dT, 0,
         0, 0, dT;

//    ROS_INFO_STREAM("dT: " << dT);
//    ROS_INFO_STREAM("Q: " << Q);
//    ROS_INFO_STREAM("F: " << F);
//    ROS_INFO_STREAM("G: " << G);
}

Eigen::Vector3d EstimatorNode::rotateAcc(){

    Eigen::Vector3d acc;

    acc(0) = u(0) - accBias(0);
    acc(1) = u(1) - accBias(1);
    acc(2) = u(2) - accBias(2);

    tf::Quaternion quat;
    quat.setX(quaternion[0]);
    quat.setY(quaternion[1]);
    quat.setZ(quaternion[2]);
    quat.setW(quaternion[3]);

    tf::Vector3 acc_tf (acc(0), acc(1), acc(2));

    Eigen::Vector3d acc_res;
    tf::vectorTFToEigen(tf::quatRotate(quat,acc_tf), acc_res);

    acc_res(2) = acc_res(2) - GRAVETAT;

//    ROS_INFO_STREAM("ACC: " << acc);
//    ROS_INFO_STREAM("ACC RES: " << acc_res);

    return acc_res;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "estimator");

  EstimatorNode estimator_node;

  ros::spin();

  return 0;
}
