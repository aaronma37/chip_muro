#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <tf/tf.h> 

// Test msg
geometry_msgs::PoseArray testPose;
geometry_msgs::Pose poseArray[10];
geometry_msgs::PoseStamped poseEst;

// Constants
const double T = 1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower_test_sample_input"); //Ros Initialize
    ros::start();
    ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)

    ros::NodeHandle n;
    ros::Publisher testPub;
    ros::Publisher poseEstPub;
    
    //testPose.poses = poseArray;

    testPub = n.advertise<geometry_msgs::PoseArray>("/path", 1000, true);
    poseEstPub = n.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);

    // Initialize test msg
    testPose.header.frame_id = "OPEN";
    for(int i = 0; i < 5; i++)
    {
        testPose.poses.push_back(poseArray[i]);
    }
    for(int i = 0; i < 5; i++)
    {
      if(i == 0)
      {
        (testPose.poses[i]).position.x = 0.01;
        (testPose.poses[i]).position.y = 0.01;
      }
      else if(i == 1)
      {
        (testPose.poses[i]).position.x = 3;
        (testPose.poses[i]).position.y = 0;  
      }
      else if(i == 2)
      {
        (testPose.poses[i]).position.x = 3;
        (testPose.poses[i]).position.y = 3;  
      }
      else if(i == 3)
      {
        (testPose.poses[i]).position.x = 0;
        (testPose.poses[i]).position.y = 3;  
      }
      else if(i == 4)
      {
        (testPose.poses[i]).position.x = 0;
        (testPose.poses[i]).position.y = 0;  
      }
      (testPose.poses[i]).position.z = 0;
    }
    poseEst.pose.position.x = 2.0;
    poseEst.pose.position.y = 2.0;
    poseEst.pose.position.z = 0.0;
    poseEst.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    
    ros::Duration d = ros::Duration(5, 0);
    d.sleep();
    
    //for(int i = 0; i < 100; i++)
    //{
        poseEstPub.publish(poseEst);
        testPub.publish(testPose);
    //}
    
    ros::shutdown();
    return 0;
}
//END
