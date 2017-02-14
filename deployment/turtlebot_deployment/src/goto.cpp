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
#include <time.h> 

//Declare Variables
double y, x, r,x1,y1,x2,y2;
double orientation;
double robVel_;
double OmegaC;
double OmegaD;
double cenx, ceny;
// Construct Node Class


class pathFollowing
{
public:
pathFollowing();
std::string this_agent_;
private:
// Methods

void poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr&);
void velocityCallback(const geometry_msgs::Twist::ConstPtr&);
void cal0Callback(const std_msgs::Float64::ConstPtr&);
void calDCallback(const std_msgs::Float64::ConstPtr&);
void updateCentroid(const geometry_msgs::PoseStamped::ConstPtr&);
// ROS stuff
ros::NodeHandle ph_, nh_;
ros::Subscriber pos_sub_;
ros::Subscriber vel_sub_;
ros::Subscriber cal0_sub_;
ros::Subscriber calD_sub_, cen_sub_;

// Other member variables

geometry_msgs::Twist robVel;
turtlebot_deployment::PoseWithName Pose;

bool got_vel_;

};

pathFollowing::pathFollowing():
/*cmd_vel_(new geometry_msgs::Twist),
*/got_vel_(false),
ph_("~"),
this_agent_()
{
ph_.param("robot_name", this_agent_,this_agent_);
ph_.param("radius", r,r);
pos_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("afterKalman", 1, &pathFollowing::poseCallback, this);
cen_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/centroidPos",1, &pathFollowing::updateCentroid, this);
}


void pathFollowing::updateCentroid(const turtlebot_deployment::PoseStamped::ConstPtr& cenPose){
cenx=cenPose->pose.position.x;
ceny=cenPose->pose.position.y;
}

void pathFollowing::poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& Pose)
{
	orientation = tf::getYaw(Pose->pose.orientation);

	x1=cos(orientation);
	y1=sin(orientation);
	x=Pose->pose.position.x;
	y=Pose->pose.position.y; 

	x2=cenx-x;
	y2=ceny-y;
	
}



int main(int argc, char **argv)
{
ros::init(argc, argv, "goto");
    ros::Rate loop_rate(50); 
time_t timer,begin,end;
ros::NodeHandle ph_("~"), nh_;
ros::Publisher u_pub_;
geometry_msgs::Twist cmd_vel_;
u_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);
pathFollowing pathFollowingk;

robVel_=0;
time(&end);
double k=1.75;
ros::spinOnce();
double u1=robVel_;
double u2=robVel_/r;
OmegaC=2;
OmegaD=1;
		while(ros::ok()){
					ros::spinOnce();
					dot = x1*x2 + y1*y2;
					det = x1*y2 - y1*x2;

					angle = atan2(det, dot);
					cout << "~~~~angle: " << angle << "\n\n";
					loop_rate.sleep();

		}
	






}
