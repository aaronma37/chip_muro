/*

Go to a position function by Aaron Ma :D

*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "PoseWithName.h"
#include <tf/tf.h>
#include <std_msgs/Float32.h>
#include <math.h>

using namespace std;



int main(int argc, char **argv)
{

ros::init(argc, argv, "goto");
ros::NodeHandle ph_;

geometry_msgs::PoseStamped fakeit;

ros::Publisher init_pub_leo = ph_.advertise<geometry_msgs::PoseStamped>("/leonardo/ekfSwitch", 1, true);
ros::Publisher init_pub_raphael = ph_.advertise<geometry_msgs::PoseStamped>("/raphael/ekfSwitch", 1, true);
ros::Publisher init_pub_bellini = ph_.advertise<geometry_msgs::PoseStamped>("/bellini/ekfSwitch", 1, true);
ros::Publisher init_pub_titian = ph_.advertise<geometry_msgs::PoseStamped>("/titian/ekfSwitch", 1, true);
ros::Publisher init_pub_michelangelo = ph_.advertise<geometry_msgs::PoseStamped>("/michelangelo/ekfSwitch", 1, true);

	   sleep(2);         //make the programme waiting for 5 secondes

for (int i=0;i<50;i++){

	fakeit.header.frame_id="leonardo";
	fakeit.pose.position.x=850;
	fakeit.pose.position.y=850;
	init_pub_leo.publish(fakeit);

	fakeit.header.frame_id="raphael";
	fakeit.pose.position.x=850;
	fakeit.pose.position.y=850;
	init_pub_raphael.publish(fakeit);

	fakeit.header.frame_id="bellini";
	fakeit.pose.position.x=850;
	fakeit.pose.position.y=850;
	init_pub_bellini.publish(fakeit);

	fakeit.header.frame_id="titian";
	fakeit.pose.position.x=850;
	fakeit.pose.position.y=850;
	init_pub_titian.publish(fakeit);

	fakeit.header.frame_id="michelangelo";
	fakeit.pose.position.x=850;
	fakeit.pose.position.y=850;
	init_pub_michelangelo.publish(fakeit);


}


}
