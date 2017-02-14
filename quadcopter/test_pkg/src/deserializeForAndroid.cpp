

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/tf.h>
#include <fstream>
#include <math.h>
#include <time.h> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sstream>

using namespace std;
const int num=50;
// Position and movement messages
geometry_msgs::PoseStamped poseEstimation[num]; // Where the Quadcopter thinks it is
geometry_msgs::PoseArray poseArrayCentroids;
geometry_msgs::Twist velEstimation[num];
geometry_msgs::PoseStamped poseGoal; // Where the Quadcopter should be
geometry_msgs::PoseStamped poseError[num]; // Difference between desired pose and current pose
tf2_msgs::TFMessage velocity; // Velocity command needed to rectify the error
geometry_msgs::Vector3 pidGain; // Store pid gain values
geometry_msgs::Vector3 poseSysId[num]; // Store best pose estimations to use for the system id
geometry_msgs::Vector3 velPoseEstX; // Used for modeling purposes
geometry_msgs::Twist pathVel; // velocity along path


// Keep track of Quadcopter state
double T = 50; // ROS loop rate

// Constants

const double PI = 3.141592653589793238463;
const double DEFAULT_KP = 0.5;
const double DEFAULT_KI = 0.01;
const double DEFAULT_KD = 0;
const double DEFAULT_KPZ = 4;
const double DEFAULT_KIZ = 0;
const double DEFAULT_KDZ = .1;
const double WINDUP_BOUND = 1.0;

// Initialize pid gains
double kp = DEFAULT_KP; // Proportional gain
double ki = DEFAULT_KI; // Integral gain
double kd = DEFAULT_KD; // Differential gain
double kpZ = DEFAULT_KPZ;
double kiZ = DEFAULT_KIZ;
double kdZ = DEFAULT_KDZ;
double sX[num];
double sY[num];
double sZ[num];
double sXprev = 0;
double sYprev = 0;
double sZprev = 0;
//double pastX = 0;
//double pastY = 0;
//double pastZ = 0;
double sliderSlope = 0.8;
double sliderGain = 0.8;
double sliderRange = 0.25;
double sliderAmp = 0.35;

// Integral windup
double windupCap;

// Kepp track of yaw to determine angular component of velocity 
double poseEstYaw[num] ; // twist or oscillation about a vertical axis
bool active[num];
bool gotPing=false;
bool centroid_deployment_active=false;
double poseGoalYaw = PI;
double poseErrYaw[num];
double poseErrYawPrev[num]; // for PID derivative term
double pastYawErr[num];

// Moving Average terms
const int numSamples = 10;
float maArrayX [numSamples] = {0}; // Array of size numSamples for moving average calculations.
float maArrayY [numSamples] = {0};
float maArrayZ [numSamples] = {0};
float maArrayYaw [numSamples] = {0};
float *maResults = new float[4];
float activeAngle=0;
int maIndex = 1;
int k=0;

// PID controller terms
geometry_msgs::PoseStamped pastError[num] ; // This is the integral term
geometry_msgs::PoseStamped poseErrorPrev[num] ; // This is used to determine the derviative term                                    

// Updates current position estimate sent by the ekf
void poseEstCallback(const tf2_msgs::TFMessage::ConstPtr& posePtr)
{
	for (int i =0;i<num;i++){
		poseArrayCentroids.poses[i].position.x = posePtr -> transforms[i].transform.translation.x;
		poseArrayCentroids.poses[i].position.y = posePtr -> transforms[i].transform.translation.y;
		poseArrayCentroids.poses[i].position.z = posePtr -> transforms[i].transform.translation.z;

		poseArrayCentroids.poses[i].orientation.x = posePtr -> transforms[i].transform.rotation.x;
		poseArrayCentroids.poses[i].orientation.y = posePtr -> transforms[i].transform.rotation.y;
		poseArrayCentroids.poses[i].orientation.z = posePtr -> transforms[i].transform.rotation.z;
		poseArrayCentroids.poses[i].orientation.w = posePtr -> transforms[i].transform.rotation.w;
	}    
}

void pingCallback(const std_msgs::Empty::ConstPtr& posePtr)
{
	 gotPing=true;
}

void messageCallback(const std_msgs::Int32MultiArray::ConstPtr& iPtr){
	if (iPtr -> layout.data_offset==1){
		centroid_deployment_active=true;
	}
	else {
		centroid_deployment_active=false;
	}
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Deserializer for Android Application"); //Ros Initialize
    ros::start();
    ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
    srand (time(NULL));
    ros::NodeHandle n;
    ros::Subscriber poseEstSub;
    ros::Publisher poseSysIdPub;
    ros::Publisher poseA;
    ros::Publisher pinger;
    ros::Subscriber pingListener;
    ros::Subscriber message_Sub;
    


    poseArrayCentroids.poses.resize(50);
	
    std_msgs::Empty j;

    poseEstSub = n.subscribe<tf2_msgs::TFMessage>("/poseEstimationC", 1, poseEstCallback);
    poseSysIdPub = n.advertise<geometry_msgs::PoseStamped>("/poseEstimationForAndroid", 1000, true);
    message_Sub = n.subscribe<std_msgs::Int32MultiArray>("/message", 1, messageCallback);
    poseA = n.advertise<geometry_msgs::PoseArray>("/toVoronoiDeployment", 1000, true);
    pinger = n.advertise<std_msgs::Empty>("/pingBack",1,true);
    pingListener = n.subscribe<std_msgs::Empty>("/ping", 1, pingCallback);
    geometry_msgs::PoseStamped poseStamped;

int i=0;
    while (ros::ok()) 
    {
        ros::spinOnce();
	if (centroid_deployment_active==true){
		poseA.publish(poseArrayCentroids);
	}	

        loop_rate.sleep();
    }
}

