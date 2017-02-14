/*

Cyclic Pursuit


*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "PoseWithName.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

double rad1,x0, z;
double yc;
double rad2,x2,y2,radN,cenx,ceny;

std::string name_, name2_;



void selfCallback(const turtlebot_deployment::PoseWithName::ConstPtr& selfPtr)
{
name_=selfPtr->name;    
x0=selfPtr->pose.position.x-cenx;
yc=selfPtr->pose.position.y-ceny;
z=yc/x0;
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
}
}

void allPoseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& posePtr)
{
    if (name_!=posePtr->name){
      x2=posePtr->pose.position.x-cenx;
      y2=posePtr->pose.position.y-ceny;
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
        }*/
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

void updateCentroid(const turtlebot_deployment::PoseWithName::ConstPtr& cenPose){
cenx=cenPose->pose.position.x;
ceny=cenPose->pose.position.y;
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "cyclic_pursuit");
ros::NodeHandle ph_, nh_;
ros::Publisher vel_pub_;
ros::Subscriber pos_sub_;
ros::Subscriber self_sub_,cen_sub_;
geometry_msgs::Twist cmd_vel_;
ros::Rate loop_rate(.2);
vel_pub_ = nh_.advertise<geometry_msgs::Twist>("velocity", 5, true);
pos_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("/all_positions", 1000,allPoseCallback);
self_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("afterKalman",1,selfCallback);
cen_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("/centroidPos",1, updateCentroid);
cmd_vel_.linear.x=100;
double k=10;
cenx=300;
ceny=200;
radN=4;
rad2=1;
rad1=1;
yc=0;
y2=0;
x0=0;
x2=0;
name2_="temp";
while (1==1){

    ros::spinOnce();
    
std::cout<<"RADIANS1: "<<rad1<<"\n";
std::cout<<"RADIANS2: "<<rad2<<"\n";
    cmd_vel_.linear.x=k*(radN);
    vel_pub_.publish(cmd_vel_);
    usleep(100000);
    
}
}
