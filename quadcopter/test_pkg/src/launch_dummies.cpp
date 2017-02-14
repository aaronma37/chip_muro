/*

Go to a position function by Aaron Ma :D

*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseStamped.h"

#include <tf/tf.h>
#include <std_msgs/Float32.h>
#include <math.h>


using namespace std;



geometry_msgs::PoseStamped fake_pose;

int main(int argc, char **argv)
{

ros::init(argc, argv, "launchdummies");
ros::NodeHandle ph_, nh_;



ros::Publisher leonardo = nh_.advertise<geometry_msgs::PoseStamped>("/leonardo/ekfSwitch", 1, true);
ros::Publisher titian = nh_.advertise<geometry_msgs::PoseStamped>("/titian/ekfSwitch", 1, true);
ros::Publisher raphael = nh_.advertise<geometry_msgs::PoseStamped>("/raphael/ekfSwitch", 1, true);
ros::Publisher michelangelo = nh_.advertise<geometry_msgs::PoseStamped>("/michelangelo/ekfSwitch", 1, true);
ros::Publisher bellini = nh_.advertise<geometry_msgs::PoseStamped>("/bellini/ekfSwitch", 1, true);
ros::Publisher massacio = nh_.advertise<geometry_msgs::PoseStamped>("/massacio/ekfSwitch", 1, true);

fake_pose.pose.position.x=0;
fake_pose.pose.position.y=0;

int i=0;
		while(ros::ok()){
			if(i<1000){
				fake_pose.header.frame_id="leonardo";
				leonardo.publish(fake_pose);

				fake_pose.header.frame_id="titian";
				titian.publish(fake_pose);

				fake_pose.header.frame_id="raphael";
				raphael.publish(fake_pose);

				fake_pose.header.frame_id="michelangelo";
				michelangelo.publish(fake_pose);

				fake_pose.header.frame_id="bellini";
				bellini.publish(fake_pose);

				fake_pose.header.frame_id="massacio";
				massacio.publish(fake_pose);
				i++;
			}
		}


}
