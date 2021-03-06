#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
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
#include <std_msgs/Float64.h>
#include <sstream>
#include <search.h>
#include <stdlib.h>
#include "VoronoiDiagramGenerator.h"
#include <tf2_msgs/TFMessage.h>
#include "CoMGenerator.h"

using namespace std;
double T=50;
float distanceThreshold =.25;
string identification;
bool gotPose=false;
const int maxNum=50;
float xValues[maxNum];
float yValues[maxNum];   
int selectedIndices[maxNum]; 
float minX = -.45, maxX = .45;    
float minY = -.45, maxY = .45;
bool active[maxNum];
geometry_msgs::Pose allPositions[maxNum];
geometry_msgs::TwistStamped aVel;

void poseCallback(const geometry_msgs::PoseArray::ConstPtr& pose)
{
	gotPose=true;

	for (int i=0;i<maxNum;i++)
	{
		if ( (pose ->poses[i].position.x!=0 || pose ->poses[i].position.y!=0) ){
			allPositions[i]=pose ->poses[i];		
			active[i]=true;	
		}
	}
	 
}

string id(int i){
identification = "not_init";
if (i==11){ identification ="dummy1";}
else if (i == 12){identification = "dummy2";}
else if (i == 13){identification = "dummy3";}
else if (i == 14){identification = "dummy4";}
else if (i == 15){identification = "dummy5";}
else if (i == 16){identification = "dummy6";}
else if (i == 17){identification = "dummy7";}
else if (i == 18){identification = "dummy8";}
else if (i == 19){identification = "dummy9";}
else if (i == 20){identification = "dummy 10";}
else if (i == 21){identification = "dummy 11";}
else if (i == 22){identification = "dummy 12";}
else if (i == 23){identification = "dummy 13";}
else if (i == 24){identification = "dummy 14";}
else if (i == 25){identification = "dummy 15";}
else if (i == 26){identification = "dummy 16";}
else if (i == 49){identification = "not_init";}
return identification;

}




////////////////////////////////////~~~~~MAIN~~~~~/////////////////////////////////////////////
int main(int argc, char **argv)
{

		ros::init(argc, argv, "Collision Avoidance"); 
		ros::start();
		ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
		ros::NodeHandle nh_;
		ros::Subscriber pos_sub_ ;
		ros::Publisher aVel_pub_ ;

		void poseCallback(const geometry_msgs::PoseArray::ConstPtr& pose);

		pos_sub_= nh_.subscribe<geometry_msgs::PoseArray>("/toVoronoiDeployment", 1000,poseCallback);
		aVel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/aVel", 1000, true);

		float distance=0;

		for (int i=0;i<maxNum;i++){
		active[i]=false;
		
		}

    while (ros::ok())
    {
	ros::spinOnce();
		aVel.twist.linear.y=0;
		aVel.twist.linear.x=0;

		if (gotPose==true){
			for (int i=0;i<maxNum;i++){
				if (active[i]==true){

					for (int j=0;j<maxNum;j++){
						if (active[j]==true && j!=i){
							distance =sqrt(	(allPositions[i].position.x-allPositions[j].position.x)*(allPositions[i].position.x-allPositions[j].position.x)+(allPositions[i].position.y-allPositions[j].position.y)*(allPositions[i].position.y-allPositions[j].position.y)	);
							if(distance< distanceThreshold){
								aVel.twist.linear.y=-(allPositions[i].position.y-allPositions[j].position.y)/distance;
								aVel.twist.linear.y=aVel.twist.linear.y-aVel.twist.linear.y*distance/distanceThreshold;
								aVel.twist.linear.x=(allPositions[i].position.x-allPositions[j].position.x)/distance;
								aVel.twist.linear.x=aVel.twist.linear.x-aVel.twist.linear.x*distance/distanceThreshold;
								aVel.header.frame_id=id(i);
								aVel.twist.angular.x=i;
							        aVel.twist.angular.y=1-distance/distanceThreshold;
								if (aVel.header.frame_id.compare("not_init")!=0){aVel_pub_.publish(aVel);}

								aVel.twist.linear.y=-aVel.twist.linear.y;
								aVel.twist.linear.x=-aVel.twist.linear.x;
								aVel.twist.angular.x=j;
								aVel.header.frame_id=id(j);
								if (aVel.header.frame_id.compare("not_init")!=0){aVel_pub_.publish(aVel);}
							}
						}
					}			
				}
				active[i]=false;
			}
			gotPose=false;
			
		}
		
    loop_rate.sleep();
    }
        
}



































