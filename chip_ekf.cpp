#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <fstream>
#include <math.h>
#include "PoseWithName.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "eigen/Eigen/Dense"
#include <vector>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <sstream>
#include <tf2_msgs/TFMessage.h>

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;

Matrix3f Q= Matrix3f::Zero();
Matrix3f R= Matrix3f::Zero();
Matrix3f W= Matrix3f::Identity();
Matrix3f I= Matrix3f::Identity();
Matrix3f P= Matrix3f::Zero();
Matrix3f H= Matrix3f::Identity();
MatrixXf X(3,1);
Matrix3f A;
Matrix3f K;
VectorXf Z(3);
turtlebot_deployment::PoseWithName goalPose;

std::string name_;
turtlebot_deployment::PoseWithNamePtr newPose_;
turtlebot_deployment::PoseWithName kill;
geometry_msgs::PoseStamped  rectified;
geometry_msgs::Pose2D location;
geometry_msgs::Quaternion  pre_theta;
ros::Publisher gl_pub_,chip_pub ;


double transformScale=900;
double transformX=301;
double transformY=247;
float lastX=0;
float lastY=0;
float lastYaw=0;
bool odom_first_use_flag=false;


const float PIXEL_SCALE=285;


bool got_pose_, stationary, active=false;
bool odom_on=true;
bool gotFirstMeasurement=false;
double theta,x,y, velo, omega;
double T =20;
double timeSinceMeasurement=100;
int counter11=0;
std::string name1_;

class getName
{
public:
getName();
std::string name1_;
private:
ros::NodeHandle ph1_, ph;

} ;
getName::getName():
ph1_("~"),
name1_("no_name")
{
ph1_.param("robot_name", name1_, name1_);
name_=name1_;
}



void orb_slam_cb(const geometry_msgs::PoseStamped::ConstPtr& posePtr)
{
    const geometry_msgs::PoseStamped& msg=*posePtr;
	if(msg.header.frame_id.compare(name1_)==0){
		gotFirstMeasurement=true;
		active=true;
		got_pose_ = true;

		std::cout<<"ORB SLAM FOR: " << name1_ << "DETECTED";

		x = msg.pose.position.y*1265;
		y = -msg.pose.position.x*1265;

		//z = -msg.transforms[0].transform.translation.y;
		pre_theta.x = msg.pose.orientation.y;
		pre_theta.y = msg.pose.orientation.x;
		pre_theta.z = -msg.pose.orientation.y;
		pre_theta.w = msg.pose.orientation.w;
		theta = tf::getYaw(pre_theta);
		theta = theta + 3.14;
	}
}

void poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& posePtr)
{
	if (posePtr->pose.position.x==0&&posePtr->pose.position.y==0){
		gotFirstMeasurement=false;

	}else{
		gotFirstMeasurement=true;
		active=true;
	}

	std::cout<<"name_ "<<name_<<"\n";
	std::cout<<posePtr->name<<"\n";

	if (posePtr->name==name_){
		got_pose_=true;
		x=posePtr->pose.position.x;
		y=posePtr->pose.position.y;
		theta = tf::getYaw(posePtr->pose.orientation)+3.14;
	}
	
}


void iptCallback(const geometry_msgs::Twist::ConstPtr& ipt){
	if (odom_on==false){
	velo=-ipt->linear.x;
	omega=ipt->angular.z;
	}

	
}

int return_sign(float to_be_assigned){
	if (to_be_assigned<0){
	return 1;
	}else{
	return -1;
	}
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	if (odom_on==true){
		if (odom_msg->pose.pose.position.x!=0 && odom_msg->pose.pose.position.y!=0){

			float newX=odom_msg->pose.pose.position.x;
			float newY=odom_msg->pose.pose.position.y;
			float xVel=odom_msg->twist.twist.linear.x;
			float tempYaw=tf::getYaw(odom_msg->pose.pose.orientation);


			if (odom_first_use_flag){
				X(0)=X(0)+PIXEL_SCALE*return_sign(xVel)*cos(X(2))*sqrt((newX-lastX)*(newX-lastX)+(newY-lastY)*(newY-lastY));
				X(1)=X(1)+PIXEL_SCALE*return_sign(xVel)*sin(X(2))*sqrt((newX-lastX)*(newX-lastX)+(newY-lastY)*(newY-lastY));
				X(2)=X(2)+(tempYaw-lastYaw);
			}

			lastX=newX;
			lastY=newY;
			lastYaw=tempYaw;
			odom_first_use_flag=true;
		}
	}

}





void fakeCallback(const geometry_msgs::PoseStamped::ConstPtr& b)
{

	active=true;
	X(0) = b -> pose.position.x;
	X(1) = b -> pose.position.y;
	X(2) = 3.14;

	x=b -> pose.position.x;
	y=b -> pose.position.y;
	theta = 3.14;

	odom_on=false;

}



int main(int argc, char **argv)
{
ros::init(argc, argv, "ekf_temp"); //Ros Initialize
ros::start();
ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
ros::Time timeBegin(0);
ros::Time timeCurrent(0);
getName getname;


double send_;

ros::NodeHandle nh_, ph_, gnh_, ph("~");
ros::Subscriber pos_sub_, orb_sub_ ;
ros::Subscriber ipt_sub_, fake_sub,odom_sub_;

ros::Publisher sf_pub_;
ros::Publisher nm_pub_;
ros::Publisher cal0_pub_;
ros::Publisher calD_pub_;
ros::Publisher measured_pub_;
ros::Publisher kalmanError;
ros::Publisher rect_pub;

ph.getParam("sendAll", send_);

name1_=getname.name1_;


void poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& pose);
void iptCallback(const geometry_msgs::Twist::ConstPtr&);
// ROS stuff
// Other member variables
got_pose_=false;
stationary=false;
Q(0,0)=0;
Q(1,1)=0;
Q(2,2)=0;
R(0,0)=.01;
R(1,1)=.01;
R(2,2)=.01;

P=Matrix3f::Zero(); //Initialize Matrix P(confidence) to be "loose"
P(0,0)=900;
P(1,1)=900;
P(2,2)=900;
H=Matrix3f::Identity();

X(0)=0;
X(1)=0;
X(2)=0;
double OmegaC=1.5;
double OmegaD=.7;
double counter12=0;
double x0=0;
double y0=0;
double ec=0;
double ed=0;
double id=.7;
double ic=1.5;
double ed0=0;
double ec0=0;
std_msgs::Float64 floatMsg, floatMsg2, ke;	
kill.name=name_;

int iTemp;
iTemp=0;

pos_sub_= nh_.subscribe<turtlebot_deployment::PoseWithName>("toKalmanfilter", 1,poseCallback);
orb_sub_= nh_.subscribe<geometry_msgs::PoseStamped>("/ORB_SLAM", 25, orb_slam_cb);


fake_sub=nh_.subscribe<geometry_msgs::PoseStamped>("ekfSwitch",10,fakeCallback);
gl_pub_ = gnh_.advertise<turtlebot_deployment::PoseWithName>("/all_positions", 1000, true);
sf_pub_= gnh_.advertise<turtlebot_deployment::PoseWithName>("afterKalman",1,true);

rect_pub = gnh_.advertise<geometry_msgs::PoseStamped>("toFormation",1,true);

chip_pub = gnh_.advertise<geometry_msgs::Pose2D>("location",1,true);

odom_sub_=nh_.subscribe<nav_msgs::Odometry>("odom",1,odom_cb);
ipt_sub_=nh_.subscribe<geometry_msgs::Twist>("cmd_vel_mux/input/navi",1,iptCallback);


nm_pub_= gnh_.advertise<turtlebot_deployment::PoseWithName>("nametest", 5);



cal0_pub_= gnh_.advertise<std_msgs::Float64>("cal0", 1,true);
calD_pub_= gnh_.advertise<std_msgs::Float64>("calD", 1,true);
calD_pub_= gnh_.advertise<std_msgs::Float64>("calD", 1,true);
kalmanError=gnh_.advertise<std_msgs::Float64>("ke",1,true);

	while (ros::ok()) {

		got_pose_=false;
		ros::spinOnce();

		if (active==true){
			if (got_pose_==true){
//only if not using cam
				R(0,0)=10000;
				R(1,1)=10000;
				R(2,2)=10000;

				R(0,0)=1;
				R(1,1)=1;
				R(2,2)=1;
			}else{
				R(0,0)=10000;
				R(1,1)=10000;
				R(2,2)=10000;
				R(0,0)=1;
				R(1,1)=1;
				R(2,2)=1;
			}

			if (!odom_first_use_flag){
				R(0,0)=1;
				R(1,1)=1;
				R(2,2)=1;
			}
			Q(0,0)=5;
			Q(1,1)=5;
			Q(2,2)=5;
		
			VectorXf Z(3);
			Matrix3f temp;

			//Stage 1
			Z << x,y,theta;



			if (odom_on==false){
				X << X(0)+velo*167/T*cos(X(2)),X(1)+velo*167/T*sin(X(2)),X(2)+omega*45/52/T;
			}

			//Stage 2
			if (got_pose_==true){
				A << 1, 0, -velo*167/T*sin(theta),0, 1,velo*167/T*cos(theta),0, 0, 1;
				P=A*P*A.transpose()+W*Q*W.transpose();
				//Stage 3
				temp=(W*P*W.transpose()+W*R*W.transpose());
				K=P*W.transpose()*temp.inverse();
				//Stage 4
				X=X+K*(Z-X);
				//Stage 5
				P=(I-K*W)*P;

				      counter12=counter11;
				      kalmanError.publish(ke);
				      x0=X(0);
				      y0=X(1);
			}

			goalPose.pose.position.x = X(0);
			goalPose.pose.position.y = X(1);
			goalPose.pose.orientation =tf::createQuaternionMsgFromYaw(X(2)+3.14);



			if (send_==1){
				gl_pub_.publish(goalPose);
				if (goalPose.pose.position.x==0&&goalPose.pose.position.y==0){
					gotFirstMeasurement=false;
				}
			}

			location.x=goalPose.pose.position.x;
			location.y=goalPose.pose.position.y;
			location.theta=X(2)+3.14;
			
			chip_pub.publish(location);
			goalPose.name=name1_;
			nm_pub_.publish(goalPose);
			sf_pub_.publish(goalPose);

			rectified.pose.position.x = (goalPose.pose.position.x-transformX)/transformScale;
			rectified.pose.position.y = -(goalPose.pose.position.y-transformY)/transformScale;
			rectified.pose.position.z = goalPose.pose.position.z/transformScale;
			rectified.pose.orientation.x = goalPose.pose.orientation.x;
			rectified.pose.orientation.y = goalPose.pose.orientation.y;
			rectified.pose.orientation.z = goalPose.pose.orientation.z;
			rectified.pose.orientation.w = goalPose.pose.orientation.w;
			
			rect_pub.publish(rectified);
		}

		loop_rate.sleep();
	}
}
//END
