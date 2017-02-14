#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "turtlesim/Pose.h"
#include "tf/tf.h"
#include "PoseWithName.h"

#include <sstream>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <time.h>

#define pi 3.14159265358979323846

// Global Variables
nav_msgs::Path path;
geometry_msgs::Pose turtlePose;
geometry_msgs::Pose goalPose;
const float stopx = 100.0;
const float ks = 0.05;    // Forward velocity
const float km = 1;  // Max Forward velocity
const float ka = 1;//0.5;                                                                            
const float kb = -3.0;//-1;
const float kc = 0.2;

/*
const int ON_LINE = 2;
const int LINE_TO_CURVE = 1;
const int ON_CURVE = 0;
*/
bool haspath = false;
const float acc_rate = ks/10;
//int* vel_density // Linear velocity desnity along the path

// Conversion from pixels to meter
double xToMeter(double x)
{
	return (x - 301)/900;
}

double yToMeter(double y)
{
	return (y - 247)/900;
}

/*
// This function calculates the linear velocity density of the path.
void linear_vel_density(void)
{
	bool on_line = false; // used to keep track of the indexes when the boundary changes

	for(int i = 0; i < path.poses.size(); i++)
	{
		if(i == path.poses.size() - 1) // avoid index out of bounds error
			break;
			
		if(getYaw(&path.poses[i].quaternion) == getYaw(&path.poses[i + 1].quaternion)) // on a straight line
		{ // FIXME; Make sure that "&" is actually needed
			vel_density[i] = ON_LINE;
		}
		else 
		{
			if(getYaw(&path.poses[i].quaternion) == getYaw(&path.poses[i - 1].quaternion) && i != 0)
				vel_density[i] = ON_LINE;
			else vel_density[i] = ON_CURVE;
		}
	}
	vel_density[i] = ON_CURVE; // last point
}
*/

// Callback to return turtle's pose
void poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& msg)
{
  turtlePose.position.x = xToMeter(msg->pose.position.x);
  turtlePose.position.y = yToMeter(msg->pose.position.y);
 // geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(msg->theta);
  double theta = tf::getYaw(msg->pose.orientation);
  if (theta < 0) theta += 2*pi;
  turtlePose.orientation.w = cos(theta/2.0);//pose_quat;
}

// Callback to return path
void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  path = *msg;
  //ROS_INFO("Path Received");
  goalPose = msg->poses[msg->poses.size()-1].pose;
  haspath = true;
  // vel_density = new int[path.poses.size()]; // Use delete[] to free memory 
  // linear_vel_density();
}

// sinc(x) function.  Required for control law
double sinc(const double x)
{
  if (abs(x)<0.000001)
    return 1;
  return sin(x)/x;
}

// return yaw from quat w
double theta(const double w)
{
  return 2*acos(w);
}

// dTheta function.  Gives dTheta from -pi to pi
double dTheta(const double t0, const double t1)
{
  double angle = t1 - t0;
  if (angle > pi)
   return dTheta(0,angle - 2*pi);
  if (angle < -pi) 
   return dTheta(0,angle + 2*pi);
  return angle;
}


// distance function.  sqrt(x^2+y^2)
double distance(const float p0x, const float p0y, const float p1x, const float p1y)
{
  return sqrt(pow((p1x - p0x),2)+pow((p1y - p0y),2));
}

// Find the distance, +/-, of turtle from path
double distancefrompath(const int index)
{
  double dx = path.poses[index].pose.position.x - turtlePose.position.x;
  double dy = path.poses[index].pose.position.y - turtlePose.position.y;
  double dt = dTheta(atan2(dy,dx), theta(path.poses[index].pose.orientation.w)); // Need to convert to use quaternion
  //return /*-dt/abs(dt)**/distance(0,0,dx,dy);
  if(dt>0)
    return distance(dx,dy,0,0);
  if(dt<0)
    return -distance(dx,dy,0,0);
  return 0;
}

// curvature function
double curvature(const int point)
{
  if( point < path.poses.size()-1 )
  {
    geometry_msgs::Pose p0 = path.poses[point].pose;
    geometry_msgs::Pose p1 = path.poses[point+1].pose;
    return dTheta(theta(p0.orientation.w),theta(p1.orientation.w))/distance(p0.position.x,p0.position.y,p1.position.x,p1.position.y);
  }
  else // You've reached the end, good job, return 0
  {
    return 0;
  }
}

// closest function.  Finds closest point on path
int closest(const float p0x, const float p0y)
{
  double lowestValue = 100;
  int lowestIndex = 0;
  //for(int i = 0; i<sizeof(path.poses)/sizeof(path.poses[0]); i++)
  for(int i = 0; i < path.poses.size(); i++)
  {
    geometry_msgs::Pose p1 = path.poses[i].pose;
    double d = distance(p0x,p0y,p1.position.x,p1.position.y);
    if (d <= lowestValue)
    {
      lowestIndex = i;
      lowestValue = d;
    }
  }
  return lowestIndex;
}

bool isTurning(double avel){
  if ( abs(avel) >= 0.0001 ){
    return true;
  }
  else {
    return false;
  }
}

// Test of line following
int main(int argc, char **argv)
{

  ros::init(argc, argv, "LineFollow");
  ros::NodeHandle nh;
  
  // Timer (for integral control)
  time_t timer;
  time_t oldtime = time(NULL);
  double seconds;
  
  
  // Publisher and Subscriber
  ros::Publisher  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1, true);

  ros::Subscriber sub = nh.subscribe<turtlebot_deployment::PoseWithName>("afterKalman",10, poseCallback);
  ros::Subscriber pathsub = nh.subscribe<nav_msgs::Path>("amcl_path", 10, pathCallback);

  // Resize path to prevent segfault during initialization.
  path.poses.resize(1);
  
  // Velocity
  geometry_msgs::Twist velocity;

  ros::Rate loop_rate(20);

  int index;
  geometry_msgs::Pose p1;
  double l, dtheta, v, c;
  double lacc, avel;

  int state = -1;
  
  while (ros::ok())
  {
  
    ros::spinOnce();

    switch (state)
    {
      case -1: // Wait for path message
        ROS_INFO("Waiting...");
        if (haspath)
          state = 0;
        break;

      
      case 0: // Line Following
        
        index = closest(turtlePose.position.x, turtlePose.position.y);
        //ROS_INFO("Index:    %d",index);
        p1 = path.poses[index].pose;
        l = distancefrompath(index);
        ROS_INFO("l:          %f",l);
        dtheta = dTheta(theta(turtlePose.orientation.w),theta(p1.orientation.w));
        ROS_INFO("quaternion p: %f",p1.orientation.w);
        ROS_INFO("theta p:      %f",theta(p1.orientation.w));
        ROS_INFO("quaternion t: %f",turtlePose.orientation.w);
        ROS_INFO("theta t:      %f",theta(turtlePose.orientation.w));
        //ROS_INFO("-kb*dtheta: %f",-kb*dtheta);
        v = ks; // Make function of distance along path (index).  Would also have to scale angular velocity.
        c = curvature(index);  // Curvature
        ROS_INFO("c:          %f",c);
        avel = -ka*v*l*sinc(dtheta) - kb*dtheta + kc*v*cos(dtheta)*c/(1+c*l); // normally (1-c*l)

        if ( !isTurning(avel) ){
	  // calculate distance from goal
	  double dfg = distance(turtlePose.position.x,
                                turtlePose.position.y,
                                goalPose.position.x,
                                goalPose.position.y );
          if ( lacc < km * dfg ){
            lacc += acc_rate;
          }
          else {
            lacc = km * dfg;
          }
        }
        else {
          lacc = 0;
        }

	// linear and angular vel
        velocity.linear.x = v + lacc;
        velocity.angular.z = avel;

        if (!isnan(velocity.angular.z) && (index < path.poses.size()-1))
          pub.publish(velocity);

        if (index >= path.poses.size()-1)
        {
          velocity.angular.z = 0;
          velocity.linear.x = 0;
          pub.publish(velocity);
          haspath = false;
          state = -1;
        }
        break;
        
    }


    loop_rate.sleep();
  }

  // delete[] vel_density;

  return 0;
}
