
/*
Moving centroid
*/

/*
Changelog:
-created

Todo:
start with moving centroid

*/

#include <iostream>

#include <fstream>

#include "std_msgs/String.h"
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "PoseWithName.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>

using namespace std;

double cenPos;
double l;
double rad2,x2,y2,radN,x0,yc;

std::string name_, name2_;


void selfCallback(const turtlebot_deployment::PoseWithName::ConstPtr& selfPtr)
{
name_=selfPtr->name;    
x0=selfPtr->pose.position.x;
yc=selfPtr->pose.position.y;
/*z=yc/x0;
rad1=atan(z);
if (x0>=0)
{
    if (yc<0){
        rad1=rad1+2*3.14;
        }
    }
else
{
    if (yc<0){
        rad1=rad1+3.14;
    }
    else
    {
        rad1=rad1+3.14;
    }
}*/
}


/*
void allPoseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& posePtr)
{
    if (name_!=posePtr->name){
      x2=posePtr->pose.position.x-350;
      y2=posePtr->pose.position.y-250;
      rad2=atan(y2/x2);
      
if (x2>=0)
{
    if (y2<0){
        rad2=rad2+2*3.14;
        }
    }
else
{
    if (y2<0){
        rad2=rad2+3.14;
    }
    else
    {
        rad2=rad2+3.14;
    }
}
      
        if(rad2<rad1){
            rad2=3.14*2-rad1+rad2;
        }
        else{
          rad2=rad2-rad1;
        }
      
        /*if (rad2<radN){
          radN=rad2;
        }*//*
        if (name2_=="temp"){
            name2_=posePtr->name;
        }
        if (name2_==posePtr->name){
        radN=rad2;
        }
        else {
            if (rad2<radN){
                name2_=posePtr->name;
                radN=rad2;
            }
        }
    }
}
*/
int main(int argc, char **argv)
{
ros::init(argc, argv, "movingCentroid");
ros::NodeHandle ph_, gnh_, nh_;
ros::Publisher cen_pub_;
std_msgs::Float64 floatMsg;
turtlebot_deployment::PoseWithName cenPose;
//ros::Subscriber pos_sub_;
ros::Subscriber self_sub_;
//geometry_msgs::Twist cmd_vel_;
ros::Rate loop_rate(.2);
cen_pub_ = gnh_.advertise<turtlebot_deployment::PoseWithName>("/centroidPos", 5, true);
//pos_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("/all_positions", 1000,allPoseCallback);
self_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("afterKalman",1,selfCallback);
//cmd_vel_.linear.x=75;
//double k=1;
//radN=1;
//rad2=1;
//rad1=1;
yc=0;
//y2=0;
x0=0;
//x2=0;
//name2_="temp";
l=0;
cenPos=250;
while (1==1){

    ros::spinOnce();
    
//std::cout<<"RADIANS1: "<<rad1<<"\n";
//std::cout<<"RADIANS2: "<<rad2<<"\n";
 if (l==1){
  cenPos=cenPos+2;
 }else{
     
     cenPos=cenPos-2;
 }
 if (cenPos<200){
     l=1;
 }
 if (cenPos>500){
     l=0;
 }
 /*
cenPose.pose.position.x = cenPos;
cenPose.pose.position.y = 250;
*/
cenPose.pose.position.x = x0;
cenPose.pose.position.y = yc;
cen_pub_.publish(cenPose);
    usleep(100000);
    
}
}
