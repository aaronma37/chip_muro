/*

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
#include <std_msgs/Bool.h>
#include <sstream>

using namespace std;

// Position and movement messages
geometry_msgs::PoseStamped poseEstimation; // Where the Quadcopter thinks it is
geometry_msgs::PoseStamped poseGoal; // Where the Quadcopter should be
geometry_msgs::PoseStamped poseError; // Difference between desired pose and current pose
geometry_msgs::Twist velocity; // Velocity command needed to rectify the error
geometry_msgs::Vector3 pidGain; // Store pid gain values
geometry_msgs::Vector3 poseSysId; // Store best pose estimations to use for the system id
geometry_msgs::Vector3 velPoseEstX; // Used for modeling purposes
std_msgs::Bool goFlight;

// Keep track of Quadcopter state
bool updatedPoseEst, updatedPoseGoal;
double T = 50; // ROS loop rate
float activeAngle=0;

// Constants
const double PI = 3.141592653589793238463;
const double DEFAULT_KP = 0.4;
const double DEFAULT_KI = 0.01;
const double DEFAULT_KD = 0.6;
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

// Integral windup
double windupCap;

// Kepp track of yaw to determine angular component of velocity 
double poseEstYaw = 0; // twist or oscillation about a vertical axis
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
int maIndex = 2;
bool gotPose=false;

// PID controller terms
geometry_msgs::PoseStamped pastError; // This is the integral term
geometry_msgs::PoseStamped poseErrorPrev; // This is used to determine the derviative term
                                          // It stores the previous poseError

// Updates current position estimate sent by the ekf
void poseEstCallback(const geometry_msgs::PoseStamped::ConstPtr& posePtr)
{
    gotPose=true;
    updatedPoseEst = true;
    poseEstimation.pose = posePtr -> pose;
    poseSysId.x = poseEstimation.pose.position.x; // Update current pose estimation data
    poseSysId.y = poseEstimation.pose.position.y;
    poseSysId.z = poseEstimation.pose.position.z;
    poseEstYaw = tf::getYaw(poseEstimation.pose.orientation) + PI;
}

// Updates goal position sent by android
void poseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& posePtr)
{
    updatedPoseGoal = true;
    poseGoal.pose = posePtr -> pose;
    poseGoalYaw = tf::getYaw(poseGoal.pose.orientation) + PI;
}

// Updates pid gain values
void pidGainCallback(const geometry_msgs::Vector3::ConstPtr& gainPtr)
{
    if(gainPtr -> x == 0.0 && gainPtr -> y == 0.0 && gainPtr -> z == 0.0)
    {
      kp = DEFAULT_KP;
      ki = DEFAULT_KI;
      kd = DEFAULT_KD;
    }
    else
    {
      kp = (double) gainPtr -> x;
      ki = (double) gainPtr -> y;
      kd = (double) gainPtr -> z;
    }
}

// Updates pid gain values for z dimension
void pidGainZCallback(const geometry_msgs::Vector3::ConstPtr& gainPtr)
{
    kpZ = (double) gainPtr -> x;
    kiZ = (double) gainPtr -> y;
    kdZ = (double) gainPtr -> z;
}

//Tells hover file when the flight file has been launched.
void flightCallback(const std_msgs::Bool::ConstPtr& goPtr)
{
    
    goFlight.data = goPtr -> data;
    
    /*
    // Flag method 2
    if( (goFlight.data = goPtr -> data) == true )
    {
      ros::shutdown();
    }
    */
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
    maResults[0] = (maResults[0]/(maIndex-1));
    maResults[1] = (maResults[1]/(maIndex-1));
    maResults[2] = (maResults[2]/(maIndex-1));
    maResults[3] = (maResults[3]/(maIndex-1));
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
    // FIXME: Tune PID constants

    double kpYaw = .5;
    double kiYaw = 0;
    double kdYaw = 0;
    
    pastError.pose.position.x += (1/T)*poseError.pose.position.x;
    pastError.pose.position.y += (1/T)*poseError.pose.position.y;
    pastError.pose.position.z += (1/T)*poseError.pose.position.z;
    pastYawErr += (1/T)*poseErrYaw;
    
    calcMoveAvg(poseError.pose.position.x - poseErrorPrev.pose.position.x,
                poseError.pose.position.y - poseErrorPrev.pose.position.y,
                poseError.pose.position.z - poseErrorPrev.pose.position.z,
                poseErrYaw - poseErrYawPrev);
    
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
    
    velocity.linear.x = (kp*poseError.pose.position.x) + (ki*pastError.pose.position.x) + ( kd*T*(maResults[0]) ); 
    velocity.linear.y = -( (kp*poseError.pose.position.y) + (ki*pastError.pose.position.y) + ( kd*T*(maResults[1]) ) );
    velocity.linear.z = (kpZ*poseError.pose.position.z) + (kiZ*pastError.pose.position.z) + ( kdZ*T*(maResults[2]) );
    
    velocity.angular.z = (kpYaw*poseErrYaw) + (kiYaw*pastYawErr) + (kdYaw*T*(maResults[3]));
    
    
    if (velocity.linear.x!=0 && velocity.linear.y!=0){
    double dummyA=1;
    double dummyO=tan(poseEstYaw+PI);
    double norm1=sqrt((velocity.linear.x*velocity.linear.x+velocity.linear.y*velocity.linear.y));

/*    double dot=velocity.linear.x*dummyA+velocity.linear.y*dummyO;
    double norm2=sqrt(1+dummyO*dummyO);
          activeAngle=acos(dot/(norm1*norm2));*/
    double dot = dummyA*velocity.linear.x + dummyO*velocity.linear.y;   
    double det = dummyA*velocity.linear.y - dummyO*velocity.linear.x;   
    double activeAngle = atan2(det, dot);
    

/*      
      if (velocity.linear.y<0){
       activeAngle=2*PI-activeAngle;
    }*/
    std::cout<<"Vx: \n"<<velocity.linear.x<<"\n\n";
    std::cout<<"Vy: \n"<<velocity.linear.y<<"\n\n";
    std::cout<<"Active Angle: \n"<<57*activeAngle<<"\n\n";
    velocity.linear.x =  norm1*cos(activeAngle);
    velocity.linear.y =  norm1*sin(activeAngle);
    }
    
    
    // For modeling purposes
    velPoseEstX.x = poseEstimation.pose.position.x;
    velPoseEstX.y = velocity.linear.x;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Quadcopter Hover: version 1, EKF pose estimations only"); //Ros Initialize
    
    /*
    // Flag method 2
    ros::start();
    ros::NodeHandle n;
    ros::Subscriber flightSub;
    flightSub = n.subscribe<std_msgs::Bool>("/flight", 1, flightCallback);
    goFlight.data = false;
    ROS_INFO("hover.cpp: START while loop");
    ros::Rate loop_rate(2);
    while(ros::ok())
    {
      ROS_INFO("hover.cpp: IN while loop");
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("hover.cpp: END while loop");
    ros::Rate loop_rate(T);
    // END flag method 2
    */
    
    ros::start();
    ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)

    ros::NodeHandle n;
    ros::Subscriber poseEstSub;
    ros::Subscriber poseGoalSub;
    ros::Subscriber pidGainSub;
    ros::Subscriber pidGainZSub;
    ros::Subscriber flightSub;
    ros::Publisher velPub;
    ros::Publisher poseSysIdPub;
    ros::Publisher velPoseEstXPub;
    

    poseEstSub = n.subscribe<geometry_msgs::PoseStamped>("/poseEstimation", 1, poseEstCallback);
    poseGoalSub = n.subscribe<geometry_msgs::PoseStamped>("/goal_pose", 1, poseGoalCallback);
    pidGainSub = n.subscribe<geometry_msgs::Vector3>("/pid_gain", 1, pidGainCallback);
    pidGainZSub = n.subscribe<geometry_msgs::Vector3>("/pid_gainZ", 1, pidGainZCallback);
    flightSub = n.subscribe<std_msgs::Bool>("/flight", 1, flightCallback);
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
    goFlight.data = false;
    velPoseEstX.z = 0;
    
    /*
    // Flag method 1
    ROS_INFO("hover.cpp: START while loop");
    while(goFlight.data == false)
    {
      ROS_INFO("hover.cpp: IN while loop");
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("hover.cpp: END while loop");
    //*/

    while (ros::ok()) 
    {
        updatedPoseEst = false;
        updatedPoseGoal = false;
        
        
        ros::spinOnce();
        if (gotPose==true){
          calculateError();
        
        PID();

        velPub.publish(velocity);
        poseSysIdPub.publish(poseSysId);
        velPoseEstXPub.publish(velPoseEstX);
        }
        gotPose=false;
        

        
        std::cout<<"--------------------------------------------------------------------";
        loop_rate.sleep();
    }
}
//END
