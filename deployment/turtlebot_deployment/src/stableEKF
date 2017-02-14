#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
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
-Report
-Finish multiple robotic intake
-Try on actual robots
-Thoughts about updating while pose has not been published
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
** covariance in our model
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
VectorXf XT(3);
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
bool got_pose_, stationary;
double theta,
x,
y;
double T =10;
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
ros::NodeHandle ph1_;
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
XT=X;
//Add new agent's matrix elements to corresponding vectors
Xv.push_back(X);
Pv.push_back(P);
XTv.push_back(XT);
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
counter11=0;
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
XT=X;
XTv.push_back(XT);std::cout<<"Measured: \n"<<Z<<"\n\n";
agentVector[iTemp].velo=ipt->linear.x;
agentVector[iTemp].omega=ipt->angular.z;
}
else{
agentVector[iTemp].velo=ipt->linear.x;
agentVector[iTemp].omega=ipt->angular.z;
}
}
int main(int argc, char **argv)
{
ros::init(argc, argv, "ekf_temp"); //Ros Initialize
ros::start();
ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
getName getname;
ofstream myfile;
const char *path="/home/turtlebot/ekfData.txt";
myfile.open(path);
myfile <<"Data";
myfile.close();
ros::NodeHandle nh_, ph_, gnh_;
ros::Subscriber pos_sub_ ;
ros::Subscriber ipt_sub_ ;
ros::Publisher gl_pub_ ;
ros::Publisher sf_pub_;
ros::Publisher nm_pub_;
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
int iTemp;
iTemp=0;
int size = robots.size();
pos_sub_= nh_.subscribe<turtlebot_deployment::PoseWithName>("toKalmanfilter", 1,poseCallback);
ipt_sub_=nh_.subscribe<geometry_msgs::Twist>("mobile_base/commands/velocity",1,iptCallback);
gl_pub_ = gnh_.advertise<turtlebot_deployment::PoseWithName>("/all_positions", 1, true);
sf_pub_= gnh_.advertise<turtlebot_deployment::PoseWithName>("afterKalman",1,true);
nm_pub_= gnh_.advertise<turtlebot_deployment::PoseWithName>("nametest", 5);
while (ros::ok()) {
got_pose_=false;
ros::spinOnce();
if (got_pose_==true){
cout<<"got_pose_: "<<got_pose_<<"\n";
R(0,0)=1;
R(1,1)=1;
R(2,2)=1;
}else{
R(0,0)=10000;
R(1,1)=10000;
R(2,2)=10000;
}
if (stationary==true){
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
XT=XTv[iTemp];
P=Pv[iTemp];
//Stage 1
Z << agentVector[iTemp].x,agentVector[iTemp].y,agentVector[iTemp].theta;
X << X(0)+agentVector[iTemp].velo*167/T*cos(X(2)),X(1)+agentVector[iTemp].velo*167/T*sin(X(2)),X(2)+.9*agentVector[iTemp].omega*57/52/T;
cout<<"Velocity: "<<agentVector[iTemp].velo*167/T<<"\n";
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
}
//Stage 6
XT=X;
//Set Vectors
Xv[iTemp]=X;
XTv[iTemp]=XT;
Pv[iTemp]=P;
turtlebot_deployment::PoseWithName goalPose;
goalPose.pose.position.x = X(0);
goalPose.pose.position.y = X(1);
goalPose.name=agentVector[iTemp].name;
goalPose.pose.orientation =tf::createQuaternionMsgFromYaw(X(2)+3.14);
//added if Statement
//if (goalPose.name!=""){
gl_pub_.publish(goalPose);
goalPose.name=name_;
nm_pub_.publish(goalPose);
sf_pub_.publish(goalPose);
//}
counter11=counter11+1;
cout<<"Counter: "<<counter11<<"\n";
cout<<"Number of Robots: "<<size<<"\n";
cout<<"Robot #: "<<iTemp<<"\n";
std::cout<<"Measured: \n"<<Z<<"\n\n";
std::cout<<"Goal Pose\n"<<goalPose<<"\n---------\n\n\n\n";
std::cout<<"--------------------------------------------------------------------";
//}
///
loop_rate.sleep();
//sleep(1/50);
}
}
}
//END
