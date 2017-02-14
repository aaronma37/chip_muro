#include "ros/ros.h"
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>

#define TOPIXELS 231
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle nh_;
  geometry_msgs::PoseStamped goalPose_;

  ros::Publisher goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, true); 

  double x = 0,
         y = 0,
         theta = 0;
  ros::Rate r(10);

  while (ros::ok())
  {
    ROS_INFO("input goal position (x, y, theta)");
    cin >> x; 
	cin >> y;
	cin >> theta;
    
	//store the position in proper format
	goalPose_.pose.position.x = x;
	goalPose_.pose.position.y = y;   
	//goalPose_.pose.orientation.w = theta;   
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
	goalPose_.pose.orientation = odom_quat;

	goal_pub_.publish(goalPose_);
    

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
