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

//Declare Variables
double y, x, r,x1,y11,x2,y22,x3,y33,cdotMag, cdotAngle;
double orientation;
double robVel_;
double OmegaC;
double OmegaD,dot,det,angle;
double cenx, ceny;
double dist=0;
double v=0;
double V=.5;
double a=0;
double A=3;
double transformScale=900;
double cdotContribution=0;
double transformX=301;
double transformY=247;
float lastcdotAngle=0;
float CTERM=.2;
float KTERM=.5;
float CANGLETERM=.5;
bool got_cdot=false;
geometry_msgs::Twist cdot;
// Construct Node Class
using namespace std;



// Other member variables

geometry_msgs::Twist robVel;
geometry_msgs::Twist lastVel;
turtlebot_deployment::PoseWithName Pose;

bool got_vel_;
bool gotInitialGoal=false;

void cdotCallback(const geometry_msgs::Twist::ConstPtr& cdotPtr){

cdot.linear=cdotPtr->linear;
x3=cdotPtr->linear.x;
y33=-cdotPtr->linear.y;
cdotMag=sqrt(x3*x3+y33*y33);
got_cdot=true;
}

void CTERM_callback(const std_msgs::Float32::ConstPtr& ctermPtr){
CTERM=ctermPtr->data*2;
}

void KTERM_callback(const std_msgs::Float32::ConstPtr& ktermPtr){
KTERM=ktermPtr->data;
}

void updateCentroid(const geometry_msgs::PoseStamped::ConstPtr& cenPose){
gotInitialGoal=true;
	if (cenPose->pose.position.z!=-1){
	cenx=cenPose->pose.position.x;
	ceny=cenPose->pose.position.y;
	}else{
	cenx=x;
	ceny=y;
	}

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
ros::Subscriber cen_sub_, cdot_sub_;
ros::Subscriber KTERM_sub;
ros::Subscriber CTERM_sub;
ros::Publisher u_pub_;

pos_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("afterKalman", 1, poseCallback);
cen_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",1, updateCentroid);
cdot_sub_ = nh_.subscribe<geometry_msgs::Twist>("cdot",1, cdotCallback);

geometry_msgs::Twist finalVel;
KTERM_sub= nh_.subscribe<std_msgs::Float32>("/voronoi/deploymentOptions/KTERM",1,KTERM_callback);
CTERM_sub= nh_.subscribe<std_msgs::Float32>("/voronoi/deploymentOptions/CTERM",1,CTERM_callback);


//CDOT INIT
cdot.linear.x=0;
cdot.linear.y=0;
cdot.linear.z=0;


//LAST VELOCITY INIT
lastVel.linear.x=0;
lastVel.angular.z=0;

u_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1, true);

		while(ros::ok()){
					ros::spinOnce();

					if (gotInitialGoal==true){

						dot = x1*x2 + y11*y22;
						det = x1*y22 - y11*x2;
						dist=sqrt(x2*x2+y22*y22);

						angle = atan2(det, dot);

						dot = x1*x3 + y11*y33;
						det = x1*y33 - y11*x3;


						cdotAngle = -atan2(det, dot);
	
							v=cos(angle)*V;
							if (dist<.01){
								v=0;
								//THIS IS A BANDAID

							}
							else if (dist< .015){
								v=v*dist/.2;
								//THIS IS A BANDAID
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
				
		
						cdotContribution=CTERM*cos(cdotAngle)*cdotMag; //4

						if (cdotContribution>.5){
							cdotContribution=.5;						
						}
						else if (cdotContribution<-.5){
							cdotContribution=-.5;						
						}

						if (abs(cos(cdotAngle))>.9){
							finalVel.linear.x=KTERM*v+cdotContribution;
							finalVel.angular.z=a;
						}
						else{
							finalVel.linear.x=KTERM*v;
							if (cdotContribution>.02){
							finalVel.angular.z=a+CANGLETERM*(cdotAngle-lastcdotAngle);
							}else{
							finalVel.angular.z=a;
							}

						}

						lastcdotAngle=cdotAngle;


 						//FILTERING
						if (finalVel.linear.x>.3){
							finalVel.linear.x=.3;
						}
						else if  (finalVel.linear.x<-.3){
							finalVel.linear.x=-.3;
						}


						if (dist < .2){
								if ((finalVel.linear.x-lastVel.linear.x)>.01){
									finalVel.linear.x=lastVel.linear.x+.02;
								}	
								else if ((finalVel.linear.x-lastVel.linear.x)<-.01){
									finalVel.linear.x=lastVel.linear.x-.02;
								}
						}
						else  {
								if ((finalVel.linear.x-lastVel.linear.x)>.05){
									finalVel.linear.x=lastVel.linear.x+.03;
								}	
								else if ((finalVel.linear.x-lastVel.linear.x)<-.05){
									finalVel.linear.x=lastVel.linear.x-.03;
								}
		
						}
								

						if ((finalVel.angular.z-lastVel.angular.z)>.025){
									finalVel.angular.z=lastVel.angular.z+.025;
								}	
								else if ((finalVel.linear.x-lastVel.linear.x)<-.025){
									finalVel.angular.z=lastVel.angular.z-.025;
								}
						
						if (abs(finalVel.angular.z)<.0001){
						finalVel.angular.z=0;			
						}
						

						//cout << "distance: " << dist<< "\n";
						/*if (got_cdot==true){

						cout << "pow: " << cos(cdotAngle)*cdotMag<< "\n";
						}*/

						


						

						lastVel=finalVel;
						u_pub_.publish(finalVel);
						loop_rate.sleep();

			}
				else{
				sleep(1);
				}

		}

}
