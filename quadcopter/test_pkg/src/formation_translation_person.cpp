#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <sstream>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"	
#include "geometry_msgs/PoseArray.h"	
#include "geometry_msgs/PoseStamped.h"	
#include <vector>
#include <iostream>
#include <algorithm>
#include <string>
#include <stdlib.h>



//This node allows for translation of formations=

using namespace std;

geometry_msgs::Point raphael, titian, bellini, leonardo, michelangelo, formation_goal, center_of_mass;
geometry_msgs::PoseStamped final;

tf2_msgs::TFMessage person_rotation;
bool rotationFlag=false;

float rotation;
float formation_active_robots=1;
//std_msgs::Float64MultiArray formation_specifications;


void raphael_cb(const geometry_msgs::PoseStamped::ConstPtr& cenPose){
	raphael.x=cenPose->pose.position.x;
	raphael.y=cenPose->pose.position.y;
	raphael.z=cenPose->pose.position.z;
}
void titian_cb(const geometry_msgs::PoseStamped::ConstPtr& cenPose){
	titian.x=cenPose->pose.position.x;
	titian.y=cenPose->pose.position.y;
	titian.z=cenPose->pose.position.z;
}
void bellini_cb(const geometry_msgs::PoseStamped::ConstPtr& cenPose){
	bellini.x=cenPose->pose.position.x;
	bellini.y=cenPose->pose.position.y;
	bellini.z=cenPose->pose.position.z;
}

void leonardo_cb(const geometry_msgs::PoseStamped::ConstPtr& cenPose){
	leonardo.x=cenPose->pose.position.x;
	leonardo.y=cenPose->pose.position.y;
	leonardo.z=cenPose->pose.position.z;
}

void michelangelo_cb(const geometry_msgs::PoseStamped::ConstPtr& cenPose){
	michelangelo.x=cenPose->pose.position.x;
	michelangelo.y=cenPose->pose.position.y;
	michelangelo.z=cenPose->pose.position.z;
}

void person_cb(const geometry_msgs::PoseStamped::ConstPtr& cenPose){
	formation_goal.x=cenPose->pose.position.x;
	formation_goal.y=-cenPose->pose.position.y;
	rotation=tf::getYaw(cenPose->pose.orientation);
}

void read_from_android(const tf2_msgs::TFMessage::ConstPtr& posePtr){

	const tf2_msgs::TFMessage& msg=*posePtr;

	//formation_goal.x=msg.transforms[0].transform.rotation.x;
	//formation_goal.y=msg.transforms[0].transform.rotation.y;
	formation_active_robots=msg.transforms[0].transform.translation.x;
	

	rotationFlag=true;
	person_rotation=msg;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher raphael_goal_publisher = n.advertise<geometry_msgs::PoseStamped>("/raphael/move_base_simple/goal", 1000);
  ros::Publisher titian_goal_publisher = n.advertise<geometry_msgs::PoseStamped>("/titian/move_base_simple/goal", 1000);
  ros::Publisher bellini_goal_publisher = n.advertise<geometry_msgs::PoseStamped>("/bellini/move_base_simple/goal", 1000);
  ros::Publisher leonardo_goal_publisher = n.advertise<geometry_msgs::PoseStamped>("/leonardo/move_base_simple/goal", 1000);
  ros::Publisher michelangelo_goal_publisher = n.advertise<geometry_msgs::PoseStamped>("/michelangelo/move_base_simple/goal", 1000);
  ros::Publisher goal_publisher = n.advertise<tf2_msgs::TFMessage>("/formationSpecifications_modified",1000);

  ros::Subscriber goal_subscriber = n.subscribe<tf2_msgs::TFMessage>("/formationSpecifications",1, read_from_android);
  ros::Subscriber raphael_goal_subscriber = n.subscribe<geometry_msgs::PoseStamped>("/raphael/relativeGoal",1, raphael_cb);
  ros::Subscriber titian_goal_subscriber = n.subscribe<geometry_msgs::PoseStamped>("/titian/relativeGoal",1, titian_cb);
  ros::Subscriber bellini_goal_subscriber = n.subscribe<geometry_msgs::PoseStamped>("/bellini/relativeGoal",1, bellini_cb);
  ros::Subscriber leonardo_goal_subscriber = n.subscribe<geometry_msgs::PoseStamped>("/leonardo/relativeGoal",1, leonardo_cb);
  ros::Subscriber michelangelo_goal_subscriber = n.subscribe<geometry_msgs::PoseStamped>("/michelangelo/relativeGoal",1, michelangelo_cb);
  ros::Subscriber person_subscriber = n.subscribe<geometry_msgs::PoseStamped>("/boticelli/toFormation",1, person_cb);


ros::Rate loop_rate(50);

//Some inits
formation_goal.x=0;
formation_goal.y=0;
formation_goal.z=0;



 while (ros::ok()){
        ros::spinOnce();


	center_of_mass.x=0;
	center_of_mass.y=0;

//	formation_goal.x=0;
//	formation_goal.y=0;
//	formation_goal.z=0;

	if (formation_active_robots>4){
	center_of_mass.x+=raphael.x;
	center_of_mass.y+=raphael.y;
	}
	if (formation_active_robots>3){
	center_of_mass.x+=leonardo.x;
	center_of_mass.y+=leonardo.y;
	}
	if (formation_active_robots>2){
	center_of_mass.x+=titian.x;
	center_of_mass.y+=titian.y;
	}
	if (formation_active_robots>1){
	center_of_mass.x+=bellini.x;
	center_of_mass.y+=bellini.y;
	}
	if (formation_active_robots>0){
	center_of_mass.x+=michelangelo.x;
	center_of_mass.y+=michelangelo.y;
	}

	center_of_mass.x=center_of_mass.x/formation_active_robots;
	center_of_mass.y=center_of_mass.y/formation_active_robots;


	if (formation_active_robots>4){
	final.pose.position.x=raphael.x+formation_goal.x-center_of_mass.x;
	final.pose.position.y=raphael.y+formation_goal.y-center_of_mass.y;
	final.pose.position.z=raphael.z+formation_goal.z-center_of_mass.z;
	raphael_goal_publisher.publish(final);

	}
	if (formation_active_robots>3){
	final.pose.position.x=leonardo.x+formation_goal.x-center_of_mass.x;
	final.pose.position.y=leonardo.y+formation_goal.y-center_of_mass.y;
	final.pose.position.z=leonardo.z+formation_goal.z-center_of_mass.z;
	leonardo_goal_publisher.publish(final);
	}
	if (formation_active_robots>2){
	final.pose.position.x=titian.x+formation_goal.x-center_of_mass.x;
	final.pose.position.y=titian.y+formation_goal.y-center_of_mass.y;
	final.pose.position.z=titian.z+formation_goal.z-center_of_mass.z;
	titian_goal_publisher.publish(final);
	}
	if (formation_active_robots>1){
	final.pose.position.x=bellini.x+formation_goal.x-center_of_mass.x;
	final.pose.position.y=bellini.y+formation_goal.y-center_of_mass.y;
	final.pose.position.z=bellini.z+formation_goal.z-center_of_mass.z;
	bellini_goal_publisher.publish(final);
	}
	if (formation_active_robots>0){

	final.pose.position.x=michelangelo.x+formation_goal.x-center_of_mass.x;
	final.pose.position.y=michelangelo.y+formation_goal.y-center_of_mass.y;
	final.pose.position.z=michelangelo.z+formation_goal.z-center_of_mass.z;
	michelangelo_goal_publisher.publish(final);
	}
    	cout<<"rotation: "<< rotation <<"\n";
	if (rotationFlag){
		person_rotation.transforms[0].transform.translation.z=rotation;
		goal_publisher.publish(person_rotation);
	}



	loop_rate.sleep();
	}
}



















