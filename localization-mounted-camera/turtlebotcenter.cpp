// subscribe to your position
// calculate how to move to center
// publish to the cmd_vel

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "tf2_msgs/TFMessage.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <tf/transform_broadcaster.h> 
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>;
//#include "LinearMath/btMatrix3x3.h"
//#include "tf/transform_datatypes.h"


//set velocity scales
double z_scale_ = .3;
double a_scale_ = .3;

//set environment variables
const double RIGHT_LIMIT = .1;
const double LEFT_LIMIT = .1;
const double TOP_LIMIT = .1;
const double BOTTOM_LIMIT = .1;
const double READJUST_MIN_ANGLE = .001;

//intialize myPosition
double x;
double y;
double z;
double my_heading;
double a;

//middle of screen
double goal_x_ = 1.675;
double goal_y_ = 1.22; 
//initalize subscribers
ros::Subscriber tf_sub;
ros::Subscriber heading_sub; 
ros::Publisher cmdpub_;
tf::TransformListener *listener = NULL;;

void callback(const std_msgs::Float32::ConstPtr& msg)
{
  my_heading = msg->data;
  ROS_INFO("I just got a heading of %f!", my_heading);
}

//void callBack(const geometry_msgs::TransformStamped::ConstPtr& msg)
//void callBack(const boost::shared_ptr<msg const>&);
void callBack(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  // get x and y locations
  //x = msg[0]->transform.translation.x;
  //y = msg->transform.translation.y;
  
  // calculate distance
  z = sqrt(pow((x-goal_x_),2) + pow((x-goal_y_),2));

  //calculate new heading
  a = atan2((goal_y_-y),(goal_x_-x)) - my_heading;

  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
  cmd->linear.x = (z - goal_x_) * z_scale_;
  cmd->angular.z = -a * a_scale_;
  cmdpub_.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "centerTurtlebot");
  ros::NodeHandle nh;

  listener = new(tf::TransformListener);

  cmdpub_ = nh.advertise<geometry_msgs::Twist> ("/mobile_base/commands/velocity", 100);
  //subscribe to someone's position to get goal_z
  //tf_sub = nh.subscribe("tf", 100, callBack);
  heading_sub = nh.subscribe("/boticellibot/heading", 10, callback);
  ros::Rate rate(10.0);

  //tf::StampedTransform transform;

    while (nh.ok()){
      tf::StampedTransform transform;
      try{
        ros::Time now = ros::Time::now();
        ROS_INFO("I'm trying to get a transform");
        listener->waitForTransform("map", "odom", now, ros::Duration(10.0) );
        listener->lookupTransform("map", "odom",
                           now, transform);
        ROS_INFO("I just saved the transform to something");
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      
      x = transform.getOrigin().x();
      y = transform.getOrigin().y();
      
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      
      tf::Quaternion q = transform.getRotation();
      tf::Matrix3x3 m(q);
      double roll, pitch;
      m.getRPY(roll, pitch, my_heading);
      
      a = atan2((goal_y_-y),(goal_x_-x)) - my_heading;
      //a = .1;
      z = sqrt(pow((x-goal_x_),2) + pow((x-goal_y_),2));
      //z = .1;http://docs.ros.org/hydro/api/tf/html/c++/classtf_1_1Transform.html
      
      if (z > .15)
    {
      if ((x<LEFT_LIMIT||x>RIGHT_LIMIT||y<BOTTOM_LIMIT||y>TOP_LIMIT)&&(a>READJUST_MIN_ANGLE)) //if turtlebot is too near edges, and it's heading is unoptimal, fix
      {
        cmd->angular.z = -a*a_scale_;
      }
      else
      {
            cmd->linear.x = z * z_scale_;
        cmd->angular.z = -a * a_scale_;
            cmdpub_.publish(cmd);
        ROS_INFO("I'm at %f,%f, with heading %f, which is %f from my goal of %f,%f, and %f degrees off target so I'm going to move with linear velocity %f and angular velocity %f", x, y, my_heading, z, goal_x_, goal_y_, a, z * z_scale_, -a * a_scale_); 
      }
    }
      else { ROS_INFO("I'm within .35m of my target so I think I give up now. You're welcome.");}
  
  rate.sleep();
  
  delete (listener);
  return 0;

  //ros::spin();  
  }
}

