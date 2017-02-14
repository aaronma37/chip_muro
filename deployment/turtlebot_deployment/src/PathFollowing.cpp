/*

Path Following algorithm from

http://www.control.utoronto.ca/people/profs/maggiore/DATA/PAPERS/CONFERENCES/ACC08_2.pdf

*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "turtlebot_deployment/PoseWithName.h"
#include <tf/tf.h>
#include <math.h>
#include <time.h> 

//Declare Variables
double x2, x1, r;
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
void updateCentroid(const turtlebot_deployment::PoseWithName::ConstPtr&);
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
vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("velocity",1, &pathFollowing::velocityCallback, this);
pos_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("afterKalman", 1, &pathFollowing::poseCallback, this);
cal0_sub_ = nh_.subscribe<std_msgs::Float64>("cal0",1, &pathFollowing::cal0Callback, this);
calD_sub_ = nh_.subscribe<std_msgs::Float64>("calD",1, &pathFollowing::calDCallback, this);
cen_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("/centroidPos",1, &pathFollowing::updateCentroid, this);
}

void pathFollowing::cal0Callback(const std_msgs::Float64::ConstPtr& OmegaC_){
OmegaC=OmegaC_->data;
}
void pathFollowing::calDCallback(const std_msgs::Float64::ConstPtr& OmegaD_){
OmegaD=OmegaD_->data;
}

void pathFollowing::velocityCallback(const geometry_msgs::Twist::ConstPtr& robVel){
robVel_= robVel->linear.x;
got_vel_ = true;
}

void pathFollowing::updateCentroid(const turtlebot_deployment::PoseWithName::ConstPtr& cenPose){
cenx=cenPose->pose.position.x;
ceny=cenPose->pose.position.y;
}

void pathFollowing::poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& Pose)
{
	orientation = tf::getYaw(Pose->pose.orientation);
	//orientation=-orientation;
	x1=Pose->pose.position.x;
	x1=x1-cenx;
	x2=Pose->pose.position.y; //centered
	x2=x2-ceny;
	
//	got_vel_=false; *Delete
	
}
int main(int argc, char **argv)
{
ros::init(argc, argv, "PathFollowing");
cenx=300;
ceny=200;
r=90;
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
while(1==1){
	//ph_.param("radius", r,r);
		//while ((time(&begin)-end)>.1){
			ros::spinOnce();
			//ADDED V PROPORTIONAL TO RADIUS
			if(sqrt(x1*x1+x2*x2)>r){
			u1=robVel_*sqrt(x1*x1+x2*x2)/r;
				k=1.75;
				u1=robVel_;
			}
			else{
				u1=robVel_;
				k=1.75;
			}
			//pathFollowingk.pathFollowing();
			
			u2=u1/r;
			//std::cout<<"initial angular velocity: \n"<<u2<<"\n\n";
			u2=u2-k*(r*x1*cos(orientation)+r*x2*sin(orientation))/167/167; //check orientation units
			u2=u2*OmegaC;
			u1=u1*OmegaD;
			if (u2>1){u2=1;}
			if (u2<-1){u2=-1;}
			
			//std::cout<<"final angular velocity: \n"<<u2<<"\n\n";
			cmd_vel_.linear.x=(u1/167);
			cmd_vel_.angular.z=(u2);
			u_pub_.publish(cmd_vel_);
		//	time(&end);
	//	}
			
			usleep(600000);
}
	
}
