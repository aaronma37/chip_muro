/*
Test
The main function of this node is to process position (pose) 
data, compute the error between desired and estimated pose, 
and then use a tuned PID controller to determine the correct 
velocity needed to minimize that error and approximate the 
goal pose.
This node subcribes to "/poseEstimation" topic where it will 
listen for any pose data coming in from the ekf filter node.
These pose coordinates are the best estimates for where the
Quadcopter is currently located.
This node subscribes to "/goalPose" topic where 
it will listen for any pose data coming from the android node. 
These pose coordinates are the desired position for the 
Quadcopter.
FIXME: This subscriber is yet to be implemented.
This node publishes to "/cmd_vel" topic where it will update 
its velocity, to be used by the ekf node for pose estimation.
*/

// We need to record using the command below so we can get a good system id
//
// we have add in controls for when yaw isnt 0. for example, we are still sending a pure x signal even when the yaw ~=0, meaning 
// that sin(yaw)*linear.x is in the y component.  we haven't accounted for that yet here.
//
// TOOLS AGAINST INTEGRAL WINDUP
// GAIN SCHEDULING 
// write a bash script to automatically launch all files

/*
This is the command to record to a txt file
rostopic echo -p /topic_name > data.txt
*/

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sstream>

using namespace std;

// Position and movement messages
geometry_msgs::PoseStamped poseEstimation; // Where the Quadcopter thinks it is
geometry_msgs::Twist velEstimation;
geometry_msgs::PoseStamped poseGoal; // Where the Quadcopter should be
geometry_msgs::PoseStamped poseError; // Difference between desired pose and current pose
geometry_msgs::Twist velocity; // Velocity command needed to rectify the error
geometry_msgs::Vector3 pidGain; // Store pid gain values
geometry_msgs::Vector3 poseSysId; // Store best pose estimations to use for the system id
geometry_msgs::Vector3 velPoseEstX; // Used for modeling purposes


// Keep track of Quadcopter state
double T = 50; // ROS loop rate

// Constants
const double PI = 3.141592653589793238463;
const double DEFAULT_KP = 0.5;
const double DEFAULT_KI = 0.01;
const double DEFAULT_KD = 0.1;
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
double sX = 0;
double sY = 0;
double sZ = 0;
double sXprev = 0;
double sYprev = 0;
double sZprev = 0;
double pastX = 0;
double pastY = 0;
double pastZ = 0;
double sliderSlope = 0.8;
double sliderGain = 0.8;
double sliderRange = 0.25;
double sliderAmp = 0.35;

// Integral windup
double windupCap;

// Kepp track of yaw to determine angular component of velocity 
double poseEstYaw; // twist or oscillation about a vertical axis
double poseGoalYaw = PI;
double poseErrYaw = 0;
double poseErrYawPrev; // for PID derivative term
double pastYawErr = 0;

// Moving Average terms
const int numSamples = 10;
float maArrayX [numSamples] = {0}; // Array of size numSamples for moving average calculations.
float maArrayY [numSamples] = {0};
float maArrayZ [numSamples] = {0};
float maArrayYaw [numSamples] = {0};
float *maResults = new float[4];
int maIndex = 1;

// PID controller terms
geometry_msgs::PoseStamped pastError; // This is the integral term
geometry_msgs::PoseStamped poseErrorPrev; // This is used to determine the derviative term
                                          // It stores the previous poseError

// Updates current position estimate sent by the ekf
void poseEstCallback(const geometry_msgs::PoseStamped::ConstPtr& posePtr)
{
    pastX = poseSysId.x;
    pastY = poseSysId.y;
    pastZ = poseSysId.z;
    
    poseEstimation.pose = posePtr -> pose;
    poseSysId.x = poseEstimation.pose.position.x; // Update current pose estimation data
    poseSysId.y = poseEstimation.pose.position.y;
    poseSysId.z = poseEstimation.pose.position.z;
    poseEstYaw = tf::getYaw(poseEstimation.pose.orientation) + PI;
}

void velocityEstCallback(const geometry_msgs::Twist::ConstPtr& twistPtr)
{
    velEstimation.linear = twistPtr->linear;
}
// Updates goal position sent by android
void poseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& posePtr)
{
    poseGoal.pose = posePtr -> pose;
    poseGoalYaw = tf::getYaw(poseGoal.pose.orientation) + PI;
}

// Updates slider gain values
void pidGainCallback(const geometry_msgs::Vector3::ConstPtr& gainPtr)
{
    sliderSlope = (double) gainPtr -> x;
    sliderGain = (double) gainPtr -> y;
    kd = (double) gainPtr -> z;
}

// Updates pid gain values for z dimension
void pidGainZCallback(const geometry_msgs::Vector3::ConstPtr& gainPtr)
{
    kpZ = (double) gainPtr -> x;
    kiZ = (double) gainPtr -> y;
    kdZ = (double) gainPtr -> z;
}

// Calculates updated error to be used by PID
void calculateError(void)
{
    poseErrorPrev.pose = poseError.pose; // Store old pose error for PID equation
    poseErrYawPrev = poseErrYaw; // Store old pose yaw for PID equation
    poseError.pose.position.x = poseGoal.pose.position.x - poseEstimation.pose.position.x;
    poseError.pose.position.y = poseGoal.pose.position.y - poseEstimation.pose.position.y;
    poseError.pose.position.z = poseGoal.pose.position.z - poseEstimation.pose.position.z;
    poseErrYaw = poseGoalYaw - poseEstYaw;
    poseError.pose.orientation = tf::createQuaternionMsgFromYaw(poseErrYaw); 
    
}

void calcMoveAvg(float newSampleX, float newSampleY, float newSampleZ, float newSampleYaw)
{
  static bool divideSampling = false;
  
  if ((maIndex - 1) == 10)
  {
    maIndex = 1;
    divideSampling = true; 
  }
  
  maArrayX[maIndex - 1] = newSampleX;
  maArrayY[maIndex - 1] = newSampleY;
  maArrayZ[maIndex - 1] = newSampleZ;
  maArrayYaw[maIndex - 1] = newSampleYaw;
  
  maResults[0] = 0;
  maResults[1] = 0;
  maResults[2] = 0;
  maResults[3] = 0;
  
  for (int i = 0; i < numSamples ; i++)
  {
    maResults[0] += maArrayX[i];
    maResults[1] += maArrayY[i];
    maResults[2] += maArrayZ[i];
    maResults[3] += maArrayYaw[i];
  }
   
  if (divideSampling != true)
  {
    maResults[0] = maResults[0]/maIndex;
    maResults[1] = maResults[1]/maIndex;
    maResults[2] = maResults[2]/maIndex;
    maResults[3] = maResults[3]/maIndex;
  }
  else
  {
    maResults[0] = (maResults[0]/numSamples);
    maResults[1] = (maResults[1]/numSamples);
    maResults[2] = (maResults[2]/numSamples);
    maResults[3] = (maResults[3]/numSamples);
  }

  maIndex++;
}

// This is the PID method
void PID(void)
{
    double kpYaw = .5;
    double kiYaw = 0;
    double kdYaw = 0;
    
    pastError.pose.position.x += (1/T)*poseError.pose.position.x;
    pastError.pose.position.y += (1/T)*poseError.pose.position.y;
    pastError.pose.position.z += (1/T)*poseError.pose.position.z;
    pastYawErr += (1/T)*poseErrYaw;
    
    // Check for integral windup
    if( sqrt( pow(poseError.pose.position.x, 2) + pow(poseError.pose.position.y, 2) ) > WINDUP_BOUND )
    {
      ki = 0;
    }
    
    if(ki == 0)
    {
      windupCap = 0;
    }
    else windupCap = 0.05/ki;
    
    if(pastError.pose.position.x > windupCap)
    {
      pastError.pose.position.x = windupCap;
    }
    else if(pastError.pose.position.x < -windupCap)
    {
      pastError.pose.position.x = -windupCap;
    }
    
    if(pastError.pose.position.y > windupCap)
    {
      pastError.pose.position.y = windupCap;
    }
    else if(pastError.pose.position.y < -windupCap)
    {
      pastError.pose.position.y = -windupCap;
    }
    
    // Store previous s value to determine derivative term
    sXprev = sX;
    sYprev = sY;
    sZprev = sZ;
    
    // Linear Sliding Mode
    sX = -sliderSlope*poseError.pose.position.x + velEstimation.linear.x;
    sY = -sliderSlope*poseError.pose.position.y + velEstimation.linear.y;
    sZ = -sliderSlope*poseError.pose.position.z + velEstimation.linear.z;
    
    /*
    // Piecewise Sliding Mode
    
    if(poseError.pose.position.x < sliderRange && poseError.pose.position.x > -sliderRange)
    {
      sX = -sliderSlope*poseError.pose.position.x + velEstimation.linear.x;
    }
    else sX = -sliderAmp*( ( poseError.pose.position.x/( sqrt(abs(poseError.pose.position.x)) ) ) + velEstimation.linear.x );
    
    if(poseError.pose.position.y < sliderRange && poseError.pose.position.y > -sliderRange)
    {
      sY = -sliderSlope*poseError.pose.position.y + velEstimation.linear.y;
    }
    else sY = -sliderAmp*( ( poseError.pose.position.y/( sqrt(abs(poseError.pose.position.y)) ) ) + velEstimation.linear.y );
    
    if(poseError.pose.position.z < sliderRange && poseError.pose.position.z > -sliderRange)
    {
      sZ = -sliderSlope*poseError.pose.position.z + velEstimation.linear.z;
    }
    else sZ = -sliderAmp*( ( poseError.pose.position.z/( sqrt(abs(poseError.pose.position.z)) ) ) + velEstimation.linear.z );
    //*/
    
    calcMoveAvg(sX - sXprev, sY - sYprev, sZ - sZprev, poseErrYaw - poseErrYawPrev);
      
    velocity.linear.x = sX*(-sliderGain) + (kd*T*(maResults[0]));
    if (velocity.linear.x > 1){
      velocity.linear.x = 1;
    }
    else if (velocity.linear.x < -1){
      velocity.linear.x = -1;
    }
    
    velocity.linear.y = sY*sliderGain + (kd*T*(maResults[1]));
    if (velocity.linear.y > 1){
      velocity.linear.y = 1;
    }
    else if (velocity.linear.y < -1){
      velocity.linear.y = -1;
    }
    
    velocity.linear.z = sZ*-(sliderGain);
    if (velocity.linear.z > 1){
      velocity.linear.z = 1;
    }
    else if (velocity.linear.z < -1){
      velocity.linear.z = -1;
    }
    
    // Coordinate transformation from global to local coordinates
    //double tempX = velocity.linear.x;
    //velocity.linear.x = velocity.linear.x*cos(poseEstYaw) + velocity.linear.y*sin(poseEstYaw);
    //velocity.linear.y = (-tempX*sin(poseEstYaw)) + velocity.linear.y*cos(poseEstYaw);
    
    velocity.angular.z = (kpYaw*poseErrYaw) + (kiYaw*pastYawErr) + (kdYaw*T*(maResults[3]));
    
    // For modeling purposes
    velPoseEstX.x = poseEstimation.pose.position.x;
    velPoseEstX.y = velocity.linear.x;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Quadcopter Hover: version 1, EKF pose estimations only"); //Ros Initialize
    ros::start();
    ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)

    ros::NodeHandle n;
    ros::Subscriber poseEstSub;
    ros::Subscriber poseGoalSub;
    ros::Subscriber velEstSub;
    ros::Subscriber pidGainSub;
    ros::Subscriber pidGainZSub;
    ros::Publisher velPub;
    ros::Publisher poseSysIdPub;
    ros::Publisher velPoseEstXPub;

    poseEstSub = n.subscribe<geometry_msgs::PoseStamped>("/poseEstimation", 1, poseEstCallback);
    velEstSub = n.subscribe<geometry_msgs::Twist>("/velocityEstimation", 1, velocityEstCallback);
    poseGoalSub = n.subscribe<geometry_msgs::PoseStamped>("/goal_pose", 1, poseGoalCallback);
    pidGainSub = n.subscribe<geometry_msgs::Vector3>("/pid_gain", 1, pidGainCallback);
    pidGainZSub = n.subscribe<geometry_msgs::Vector3>("/pid_gainZ", 1, pidGainZCallback);
    velPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000, true);
    poseSysIdPub = n.advertise<geometry_msgs::Vector3>("/sys_id", 1000, true);
    velPoseEstXPub = n.advertise<geometry_msgs::Vector3>("/vel_poseEstX", 1000, true);

    // Initialize msgs
    poseGoal.pose.position.x = 0;
    poseGoal.pose.position.y = 0;
    poseGoal.pose.position.z = 0;
    poseError.pose.position.x = 0;
    poseError.pose.position.y = 0;
    poseError.pose.position.z = 0;
    velocity.angular.x = 1;
    velocity.angular.y = 0;
    pastError.pose.position.x = 0;
    pastError.pose.position.y = 0;
    pastError.pose.position.z = 0;

    velPoseEstX.z = 0;

    while (ros::ok()) 
    {
        
        ros::spinOnce();
        
        calculateError();
        
        PID();

        velPub.publish(velocity);
        poseSysIdPub.publish(poseSysId);
        velPoseEstXPub.publish(velPoseEstX);

        std::cout<<"Twist: \n"<<velEstimation.linear<<"\n\n";
        std::cout<<"--------------------------------------------------------------------";
        loop_rate.sleep();
    }
}
//END
