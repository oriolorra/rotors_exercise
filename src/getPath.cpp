#include "getPath.h"

bool isKF = false;
bool isGPS = false;
bool isGT = false;


GetPathNode::GetPathNode(){
    ros::NodeHandle nh;

    KF_sub_  = nh.subscribe("/firefly/pose",            1,  &GetPathNode::poseKFCallback, this);
    GPS_sub_ = nh.subscribe("/firefly/fake_gps/pose",  1, &GetPathNode::fakeGPSCallback, this);
    GT_sub_  = nh.subscribe("/firefly/groun_truth/pose",            1, &GetPathNode::groundTruthCallback, this);

    KFPath_pub_  =  nh.advertise<nav_msgs::Path>("/firefly/path/kf", 1);
    GPSPath_pub_ =  nh.advertise<nav_msgs::Path>("/firefly/path/gps", 1);
    GTPath_pub_  =  nh.advertise<nav_msgs::Path>("/firefly/path/ground_truth", 1);

    timer_ = nh.createTimer(ros::Duration(0.1), &GetPathNode::TimedCallback, this);
}

GetPathNode::~GetPathNode(){ }

void GetPathNode::poseKFCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg){
    isKF = true;

    msgPose_.header.stamp = pose_msg->header.stamp;
    msgPose_.header.seq = pose_msg->header.seq;
    msgPose_.header.frame_id = "world";
    msgPose_.pose = pose_msg->pose;

    msgKFPath_.header.stamp = pose_msg->header.stamp;
    msgKFPath_.header.seq = pose_msg->header.seq;
    msgKFPath_.header.frame_id =pose_msg->header.frame_id;

    msgKFPath_.poses.push_back(msgPose_);
}


void GetPathNode::fakeGPSCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg){

    isGPS = true;

    msgPose_.header.stamp = pose_msg->header.stamp;
    msgPose_.header.seq = pose_msg->header.seq;
    msgPose_.header.frame_id = "world";
    msgPose_.pose = pose_msg->pose;

    msgGPSPath_.header.stamp = pose_msg->header.stamp;
    msgGPSPath_.header.seq = pose_msg->header.seq;
    msgGPSPath_.header.frame_id =pose_msg->header.frame_id;

    msgGPSPath_.poses.push_back(msgPose_);
}

void GetPathNode::groundTruthCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg){

    isGT = true;

    msgPose_.header.stamp = pose_msg->header.stamp;
    msgPose_.header.seq = pose_msg->header.seq;
    msgPose_.header.frame_id = "world";
    msgPose_.pose = pose_msg->pose;

    msgGTPath_.header.stamp = pose_msg->header.stamp;
    msgGTPath_.header.seq = pose_msg->header.seq;
    msgGTPath_.header.frame_id =pose_msg->header.frame_id;

    msgGTPath_.poses.push_back(msgPose_);
}

void GetPathNode::TimedCallback(const ros::TimerEvent &e){

    if(isKF){
        isKF = false;
        KFPath_pub_.publish(msgKFPath_);

    }else if(isGPS){
        isGPS = false;
        GPSPath_pub_.publish(msgGPSPath_);

    }else if(isGT){
        isGT = false;
        GTPath_pub_.publish(msgGTPath_);
    }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_node");

  GetPathNode path_node;

  ros::spin();

  return 0;
}
