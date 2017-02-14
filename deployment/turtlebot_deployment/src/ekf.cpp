#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
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
/* -----------------------------------------------------------------------------------------
CREDIT
Credit to
Eigen for their opensource linear algebra library and headers
Allen for previous iteration of Kalman filter
ROS opensource
*/
/*
Notes for meeting-
-NEED TO FIX THE LAUNCH PROBLEM (SENDALL SEND SEND_)
*/
/*
Changelog
To Do
-State noise covariance needs to be determined
-state dynamics need to be determined
-Independent Timer
-----------------------------------------------------------------------------------------
BEGIN
*/
using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
//***Remove later
/*-----------------------------------------------------------------------------------------
** Initialization Block
**
** Initialize Q,R,W, I(identity) as a permanent matrices determined by
** covariance in our modeladam lambert what do you want from me
**
** Initialize P,H,X,XT,A,K,Z as matrices that vary
** during iterations
*/
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
vector<Vector3f> Xv;
vector<Vector3f> XTv;
vector<Matrix3f> Av;
vector<Matrix3f> Kv;
vector<Matrix3f> Pv;
vector<Matrix3f> Hv;
vector<string> robots; //Initialize a vector of robot names
std::string name_;
turtlebot_deployment::PoseWithNamePtr newPose_;
turtlebot_deployment::PoseWithName kill;
geometry_msgs::PoseStamped  rectified;
std_msgs::Header measurementF;
ros::Publisher gl_pub_ ;

double transformScale=900;
double transformX=301;
double transformY=247;
float lastX=0;
float lastY=0;
float lastYaw=0;
bool odom_first_use_flag=false;


const float PIXEL_SCALE=285;


bool got_pose_, stationary;
bool gotFirstMeasurement=false;
double theta,
x,
y;
double T =20;
double timeSinceMeasurement=100;
int counter11=0;
class agent{
public:
float x, y, theta, velo, omega, t;
string name;
};
vector<agent> agentVector;
class getName
{
public:
getName();
private:
ros::NodeHandle ph1_, ph;
std::string name1_;
} ;
getName::getName():
ph1_("~"),
name1_("no_name")
{
ph1_.param("robot_name", name1_, name1_);
name_=name1_;
}
void poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& posePtr)
{
if (posePtr->pose.position.x==0&&posePtr->pose.position.y==0){
	gotFirstMeasurement=false;
}else{
	gotFirstMeasurement=true;
}
	


std::cout<<"name_ "<<name_<<"\n";
std::cout<<posePtr->name<<"\n";
int size = robots.size();
int iTemp;
agent a; //Temp agent used if a new agent is introduced
if (posePtr->name==name_){
iTemp=0;

got_pose_=true;
if (iTemp==size)
{
//Add elements to necessary vectors
robots.push_back(posePtr->name); //Adds robot name element to vector<string>robots
agentVector.push_back(a); //Adds class "agent" element to vector<agent>agentVector
//Initialize new detected robot matrices and state
P=Matrix3f::Zero(); //Initialize Matrix P(confidence) to be "loose"
P(0,0)=900;
P(1,1)=900;
P(2,2)=900;
X<<1,2,3; //Define initial position

//Add new agent's matrix elements to corresponding vectors
Xv.push_back(X);
Pv.push_back(P);

//Set new agent's position
agentVector[iTemp].name=posePtr->name;
agentVector[iTemp].x=posePtr->pose.position.x;
agentVector[iTemp].y=posePtr->pose.position.y;
agentVector[iTemp].theta = tf::getYaw(posePtr->pose.orientation)+3.14;

}
else{
/*
** This block runs if posePtr->name was found in found, ie if robot's id was previously detected.
*/
std::cout<<"pass";
//Set found agent's position
agentVector[iTemp].x=posePtr->pose.position.x;
agentVector[iTemp].y=posePtr->pose.position.y;
agentVector[iTemp].theta = tf::getYaw(posePtr->pose.orientation)+3.14;
}
}
else{
}
}


void iptCallback(const geometry_msgs::Twist::ConstPtr& ipt)
{
//Determine if subscribed .name is already in group. If not, this initiallizes a new one
int size = robots.size();
int iTemp;
iTemp=0;
agent a;
/* -----------------------------------------------------------------------------------------
** Same iTemp search algorithm as in poseCallback
** ***This algorithm may become a separate function
*/
if (size==0){
robots.push_back(name_);
agentVector.push_back(a);
agentVector[iTemp].name=name_;
P=Matrix3f::Zero();
P(0,0)=900;
P(1,1)=900;
P(2,2)=900;
H=Matrix3f::Identity();
Pv.push_back(P);
Hv.push_back(H);
X<<1,2,3;
Xv.push_back(X);

agentVector[iTemp].velo=-ipt->linear.x;
agentVector[iTemp].omega=ipt->angular.z;
std::cout<<"\nMeasured Angular Velocity: "<<ipt->angular.z<<"\n\n";
}
else{
agentVector[iTemp].velo=-ipt->linear.x;
agentVector[iTemp].omega=ipt->angular.z;
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





void fakeCallback(const geometry_msgs::PoseStamped::ConstPtr& b)
{
	//if (b->data==true){

	//}
//got_pose_=true;

if (gotFirstMeasurement==false){
agent a; //Temp agent used if a new agent is introduced
robots.push_back(b->header.frame_id); //Adds robot name element to vector<string>robots
agentVector.push_back(a); //Adds class "agent" element to vector<agent>agentVector
//Initialize new detected robot matrices and state
P=Matrix3f::Zero(); //Initialize Matrix P(confidence) to be "loose"
P(0,0)=900;
P(1,1)=900;
P(2,2)=900;
X<<b -> pose.position.x, b->pose.position.y, 3.14; //Define initial position

//Add new agent's matrix elements to corresponding vectors

Xv.push_back(X);
Pv.push_back(P);
agentVector[0].name=b->header.frame_id;
agentVector[0].x=b -> pose.position.x;
agentVector[0].y=b -> pose.position.y;
agentVector[0].theta = 3.14;

gotFirstMeasurement=true;
	
}

}



int main(int argc, char **argv)
{
ros::init(argc, argv, "ekf_temp"); //Ros Initialize
ros::start();
ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
ros::Time timeBegin(0);
ros::Time timeCurrent(0);
getName getname;
ofstream myfile;
const char *path="/home/turtlebot/ekfData.txt";
double send_;
bool odom_on=false;
myfile.open(path);
myfile <<"Data";
myfile.close();
ros::NodeHandle nh_, ph_, gnh_, ph("~");
ros::Subscriber pos_sub_ ;
ros::Subscriber ipt_sub_, fake_sub ;

ros::Publisher sf_pub_;
ros::Publisher nm_pub_;
ros::Publisher cal0_pub_;
ros::Publisher calD_pub_;
ros::Publisher measured_pub_;
ros::Publisher kalmanError;
ros::Publisher rect_pub;

ph.getParam("sendAll", send_);
ph.getParam("odometry",odom_on);

//REMOVE LATER
odom_on=false;

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
int size = robots.size();
pos_sub_= nh_.subscribe<turtlebot_deployment::PoseWithName>("toKalmanfilter", 1,poseCallback);

fake_sub=nh_.subscribe<geometry_msgs::PoseStamped>("ekfSwitch",10,fakeCallback);
gl_pub_ = gnh_.advertise<turtlebot_deployment::PoseWithName>("/all_positions", 1000, true);
sf_pub_= gnh_.advertise<turtlebot_deployment::PoseWithName>("afterKalman",1,true);

rect_pub = gnh_.advertise<geometry_msgs::PoseStamped>("toFormation",1,true);

if (odom_on){
ipt_sub_=nh_.subscribe<nav_msgs::Odometry>("odom",1,odom_cb);
}else{
ipt_sub_=nh_.subscribe<geometry_msgs::Twist>("cmd_vel_mux/input/navi",1,iptCallback);
}

nm_pub_= gnh_.advertise<turtlebot_deployment::PoseWithName>("nametest", 5);
measured_pub_=nh_.advertise<std_msgs::Header>("/measurementFlag",1);


cal0_pub_= gnh_.advertise<std_msgs::Float64>("cal0", 1,true);
calD_pub_= gnh_.advertise<std_msgs::Float64>("calD", 1,true);
calD_pub_= gnh_.advertise<std_msgs::Float64>("calD", 1,true);
kalmanError=gnh_.advertise<std_msgs::Float64>("ke",1,true);

measurementF.frame_id=name_;
while (ros::ok()) {
	got_pose_=false;
	ros::spinOnce();

	if (gotFirstMeasurement==true){
		if (got_pose_==true){
		R(0,0)=1;
		R(1,1)=1;
		R(2,2)=1;
		}else{
		R(0,0)=10000;
		R(1,1)=10000;
		R(2,2)=10000;
		}
		if (stationary==true || (agentVector[iTemp].x==0&&agentVector[iTemp].y==0)){	
		Q(0,0)=0;
		Q(1,1)=0;
		Q(2,2)=0;
		}else{
		Q(0,0)=5;
		Q(1,1)=5;
		Q(2,2)=5;
		}
		if (0<robots.size()){
		///
		VectorXf Z(3);
		Matrix3f temp;
		P=Pv[iTemp];
		//Stage 1
		Z << agentVector[iTemp].x,agentVector[iTemp].y,agentVector[iTemp].theta;
		if (!odom_on){
			X << X(0)+agentVector[iTemp].velo*167/T*cos(X(2)),X(1)+agentVector[iTemp].velo*167/T*sin(X(2)),X(2)+agentVector[iTemp].omega*45/52/T;
		}

//		cout<<"Velocity: "<<agentVector[iTemp].velo*167/T<<"\n";
		//Stage 2
		if (got_pose_==true){
		A << 1, 0, -agentVector[iTemp].velo*167/T*sin(agentVector[iTemp].theta),0, 1,agentVector[iTemp].velo*167/T*cos(agentVector[iTemp].theta),0, 0, 1;
		P=A*P*A.transpose()+W*Q*W.transpose();
		//Stage 3
		temp=(W*P*W.transpose()+W*R*W.transpose());
		K=P*W.transpose()*temp.inverse();
		//Stage 4
		X=X+K*(Z-X);
		//Stage 5
		P=(I-K*W)*P;

		      counter12=counter11;
		      if (id<-25){id=-25;}
		      if (id>25){id=25;}
		      if (ic<-25){ic=-25;}
		      if (ic>25){ic=25;}
		      floatMsg.data=OmegaC;
		      floatMsg2.data=OmegaD;
		      cal0_pub_.publish(floatMsg);
		      calD_pub_.publish(floatMsg2);
		      kalmanError.publish(ke);
		      x0=X(0);
		      y0=X(1);
		      
		    //}
		timeCurrent=timeBegin;
		}
		else{
		timeCurrent=timeCurrent+ros::Duration(1);		
		}


		measurementF.stamp=timeCurrent;
		//Set Vectors
		Xv[iTemp]=X;
		Pv[iTemp]=P;
		turtlebot_deployment::PoseWithName goalPose;
		goalPose.pose.position.x = X(0);
		goalPose.pose.position.y = X(1);
		goalPose.name=agentVector[iTemp].name;
		goalPose.pose.orientation =tf::createQuaternionMsgFromYaw(X(2)+3.14);
		//added if Statement
		//if (goalPose.name!=""){
		if (send_==1){
		gl_pub_.publish(goalPose);
		if (goalPose.pose.position.x==0&&goalPose.pose.position.y==0){
			gotFirstMeasurement=false;
		}
		}
		goalPose.name=name_;
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



		
		measured_pub_.publish(measurementF);
		//}
		counter11=counter11+1;
//		cout<<"Counter: "<<counter11<<"\n";
//		cout<<"Number of Robots: "<<size<<"\n";
//		cout<<"Robot #: "<<iTemp<<"\n";
//		std::cout<<"Measured: \n"<<Z<<"\n\n";
//		std::cout<<"OmegaC\n"<<OmegaC<<"\n---------\n";
//		std::cout<<"OmegaD\n"<<OmegaD<<"\n---------\n";
//		std::cout<<"Goal Pose\n"<<goalPose<<"\n---------\n\n\n\n";

//		std::cout<<"--------------------------------------------------------------------";
		//}
		///
		loop_rate.sleep();
		//sleep(1/50);
		}
	}
	else{
		sleep(1);
		gl_pub_.publish(kill);
	}
}
}
//END
