/*

Path Following algorithm from

http://www.control.utoronto.ca/people/profs/maggiore/DATA/PAPERS/CONFERENCES/ACC08_2.pdf

*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <turtlebot_deployment/PoseWithName.h>
#include <tf/tf.h>
#include <math.h>

//Declare Variables
double y, x, r,x1,y11,x2,y22;
double orientation;
double robVel_;
double OmegaC;
double OmegaD,dot,det,angle;
double cenx, ceny;
double dist=0;
double v=0;
double V=.2;
double a=0;
double A=1.5;
double transformScale=900;
double transformX=301;
double transformY=247;
// Construct Node Class
using namespace std;



// Other member variables

geometry_msgs::Twist robVel;
turtlebot_deployment::PoseWithName Pose;

bool got_vel_;



void updateCentroid(const geometry_msgs::PoseStamped::ConstPtr& cenPose){
cenx=cenPose->pose.position.x;
ceny=cenPose->pose.position.y;
}

void poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& Pose)
{
	orientation = tf::getYaw(Pose->pose.orientation);
	x1=cos(orientation);
	y11=sin(orientation);
	x=(Pose->pose.position.x-transformX)/transformScale; 
	y=(Pose->pose.position.y-transformY)/transformScale;

	x2=cenx-x;
	y22=ceny-y;
	
}



int main(int argc, char **argv)
{

ros::init(argc, argv, "goto");
ros::NodeHandle ph_, nh_;
ros::Rate loop_rate(50); 
ros::Subscriber pos_sub_;
ros::Subscriber cen_sub_;
ros::Publisher u_pub_;

pos_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("donatello/afterKalman", 1, poseCallback);
cen_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("donatello/move_base_simple/goal",1, updateCentroid);
geometry_msgs::Twist finalVel;

u_pub_ = nh_.advertise<geometry_msgs::Twist>("/donatello/cmd_vel_mux/input/navi", 1, true);

		while(ros::ok()){
					ros::spinOnce();

					dot = x1*x2 + y11*y22;
					det = x1*y22 - y11*x2;
					dist=sqrt(x2*x2+y22*y22);

					angle = atan2(det, dot);
	
						v=cos(angle)*V;
						if (dist<.07){
							v=0;
						}
						else if (dist< .2){
							v=v*dist/.2;
						}
						else{

						}
					

					if (cos(angle)>0){
						if (abs(angle*180/3.14)<45){
							a=A*angle;
						}
						else{
							a=A;
						}
					}
					else if (angle*180/3.14>90){
						if (abs(angle*180/3.14)>135){
							a=A*(angle-3.14);
						}
						else{
							a=A;
						}
					}
					else if (angle*180/3.14<-90){
						if (abs(angle*180/3.14)>135){
							a=A*(angle+3.14);
						}
						else{
							a=A;
						}
					}
					





					if (dist<.07){
						a=0;
					}
		
					finalVel.linear.x=v;
					finalVel.angular.z=a;

					cout << "distance: " << dist<< "\n";
					cout << "angle: " << angle*180/3.14<< "\n\n";
					if (A<abs(a)){
					cout << "WARNING: OVER ANGULAR  \n\n";
					}
					u_pub_.publish(finalVel);
					loop_rate.sleep();

		}

}
