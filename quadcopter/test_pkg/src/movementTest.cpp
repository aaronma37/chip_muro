
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <sstream>
#include <tf/tf.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"	
#include "geometry_msgs/PoseStamped.h"	
#include <vector>
#include <std_msgs/Empty.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <stdlib.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "movement test");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
  ros::Publisher chatter_pub2 = n.advertise<std_msgs::Empty>("/ardrone/land", 1000);
  ros::Publisher chatter_pub3 = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  std_msgs::Empty myMsg;
  geometry_msgs::Twist twist;
  twist.linear.x=.05;
  twist.linear.y=0;
  twist.linear.z=0;
  twist.angular.y=0;
  twist.angular.z=0;
  twist.angular.x=1;

ros::Rate loop_rate(200);
int count =0;
chatter_pub.publish(myMsg);
 while (ros::ok()){
  
  if (count > 7000){
    chatter_pub2.publish(myMsg);
  }
  else if (count >5000){
    twist.linear.x=0;
    chatter_pub3.publish(twist);
  }
  else if (count >4000){
    twist.linear.x=-.05;
    chatter_pub3.publish(twist);
  }
  else if (count >3000){
    chatter_pub3.publish(twist);
  }
  else{
    chatter_pub.publish(myMsg);
  }
  std::cout<<"\n Count: "<<count<<"\n";
  count=count+1;
  loop_rate.sleep();
  }
}
