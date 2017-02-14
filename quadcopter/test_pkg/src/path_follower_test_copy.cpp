/*

This node processes an array of pose and publishes necessary 
goal pose in order to execute a path following algorithm.

*/

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <tf/tf.h> 

// Position data
geometry_msgs::PoseStamped poseEst; 

// Path data
geometry_msgs::PoseArray pathPose;
int lastPointOnPathIndex;

// Controller data
geometry_msgs::PoseStamped goalPose;
geometry_msgs::Twist constVelTerm;

// Constants
const double PI = 3.141592653589793238463;
const double BOUNDARY_RADIUS = 0.1;
const int T = 50;
const int NUM_ITERATIONS = 10;
const double PATH_VEL = .1;
const double LINE_DIST_RANGE = .9;



// Interpolation data
int closestPointIndex = 0;
int prevClosestPointIndex = 0;
geometry_msgs::PoseStamped closestPointOnLine; // pose of closest point on interpolation line
geometry_msgs::PoseStamped nextPointClosest; // pose of next point on path
double midpoints[2] = {0,0};
double line_dist = 0;
double line_dist_trav = 0;

// Flags
bool newPath;
bool isOpenLoop; 

// Updates current position estimate sent by the ekf
void poseEstCallback(const geometry_msgs::PoseStamped::ConstPtr& posePtr)
{
    poseEst.pose = posePtr -> pose;
}

// Determine type of path and unpack array of pose
void pathCallback(const geometry_msgs::PoseArray::ConstPtr& pathPtr)
{
  newPath = true;
  
  // Process path data
  if( (pathPtr -> header.frame_id).compare("OPEN") == 0) // check if open loop
  {
    isOpenLoop = true;
    pathPose = *pathPtr;
  }
  else
  {
    isOpenLoop = false;
    pathPose = *pathPtr;
  }
}

double distanceFormula (double x3, double x2, double y3, double y2)
{
  double c = 0;
  c = sqrt ( pow(x3 - x2, 2) + pow(y3 - y2, 2) );
  return c;
}

void calculateMidPoints(double x3, double x2, double y3, double y2)
{
    midpoints[0] = (x3+x2)/2 ;
    midpoints[1] = (y3+y2)/2 ;
}

// This function will identify the closest point on the path to the quadcopter
void calcClosestPointOnPath (void)
{
  double closestDistance = 0; // distance from pose estimation to closest point on the path
  double tempClosestDistance = 0;
  
  prevClosestPointIndex = closestPointIndex;
  for(int i = 0; i <= lastPointOnPathIndex; i++)
  {
    if ( (pathPose.poses[i].position.x == 0) && (pathPose.poses[i].position.y == 0) )
        {
            continue;
        }
    tempClosestDistance = distanceFormula ( pathPose.poses[i].position.x, poseEst.pose.position.x, 
                                        pathPose.poses[i].position.y, poseEst.pose.position.y );

    if ( (closestDistance == 0) || (closestDistance > tempClosestDistance) )
      { 
        closestDistance = tempClosestDistance;
        closestPointIndex = i;
      }
  }
  
  /*if(isOpenLoop && (closestPointIndex != prevClosestPointIndex) )
  {
  	if(closestPointIndex != prevClosestPointIndex + 1)
  	{
  		closestPointIndex = prevClosestPointIndex + 1;
  	
  	}
  }*/
}

// Outputs a constant velocity term for the sliding mode controller
void calcConstVelTerm(void)
{
    double globalAngle = 0;
    double vector1[2] = {1, 0};
    double vector2[2] = {nextPointClosest.pose.position.x - closestPointOnLine.pose.position.x, 
                        nextPointClosest.pose.position.y - closestPointOnLine.pose.position.y};
    double dotProduct = (vector1[0]*vector2[0]) + (vector1[1]*vector2[1]);
    double absProduct = distanceFormula(nextPointClosest.pose.position.x, closestPointOnLine.pose.position.x,
                                        nextPointClosest.pose.position.y, closestPointOnLine.pose.position.y);
    
    /*if(absProduct == 0)
    {
    	std::cout << "---------------------------------------------------------------------\n";
    	std::cout << "Closest Point(reference):\nIndex: "<< closestPointIndex << "\n" << closestPointOnLine << "\n\n";
        std::cout << "Next Point:\n" << nextPointClosest << "\n";
        std::cout << "---------------------------------------------------------------------\n\n";
    }*/
    
    //if(absProduct != 0)
    //{
    	globalAngle = acos(dotProduct/absProduct);
    //} 
    //else globalAngle = 0; // FIXME: check output
    
    if(nextPointClosest.pose.position.y - closestPointOnLine.pose.position.y >= 0) // Check if angle is over 180 degrees
    {
        constVelTerm.linear.x = cos(globalAngle)*PATH_VEL;
        constVelTerm.linear.y = -sin(globalAngle)*PATH_VEL;
    }
    else
    {
        constVelTerm.linear.x = cos(2*PI - globalAngle)*PATH_VEL;
        constVelTerm.linear.y = -sin(2*PI - globalAngle)*PATH_VEL;
    }
}

// determines whether the next interpolated line segment along the path should be used
// by checking the distance the quadcopter has traveled along the line
void checkDistanceTraveled(void)
{
   if(closestPointIndex != prevClosestPointIndex)
    {
    	line_dist =  distanceFormula ( pathPose.poses[closestPointIndex].position.x, pathPose.poses[closestPointIndex + 1].position.x, 
                                        pathPose.poses[closestPointIndex].position.y, pathPose.poses[prevClosestPointIndex + 1].position.y );
        line_dist_trav =  distanceFormula ( closestPointOnLine.pose.position.x, pathPose.poses[closestPointIndex].position.x, 
                                        closestPointOnLine.pose.position.y, pathPose.poses[closestPointIndex].position.y );
        if( (line_dist_trav/line_dist) < LINE_DIST_RANGE )
        {
        	closestPointIndex = prevClosestPointIndex;
        	return;
        }
        //else closestPointIndex++;
    }
}

// Interpolates to find closest point on the path using the bisection method
void findClosestPointOnLine(void)
{
    double point1[2] = {0,0};
    double point2[2] = {0,0};
    
    //if(closestPointOnLine.pose.position.x != 0 && closestPointOnLine.pose.position.y != 0) // skip first iteration
    //{
    	//checkDistanceTraveled(); 
    //}
    
    /*if(prevClosestPointIndex > closestPointIndex)
    {
    	closestPointIndex = prevClosestPointIndex + 1;
    }*/
    
    point1[0] = pathPose.poses[closestPointIndex].position.x;
    point1[1] = pathPose.poses[closestPointIndex].position.y;
    point2[0] = pathPose.poses[closestPointIndex + 1].position.x;
    point2[1] = pathPose.poses[closestPointIndex + 1].position.y;
    nextPointClosest.pose.position.x = point2[0];
    nextPointClosest.pose.position.y = point2[1];
    
    double distance1 = 0;
    double distance2 = 0;
    for(int i = 0; i<NUM_ITERATIONS; i++)
    {
        distance1 = distanceFormula(poseEst.pose.position.x, point1[0], poseEst.pose.position.y, point1[1]);
        distance2 = distanceFormula(poseEst.pose.position.x, point2[0], poseEst.pose.position.y, point2[1]);
        calculateMidPoints (point2[0], point1[0], point2[1], point1[1]);
        if(distance1 < distance2)
        {
            point2[0] = midpoints[0];
            point2[1] = midpoints[1];
        }
        else
        {
            point1[0] = midpoints[0];
            point1[1] = midpoints[1];
        }
    }
    
    closestPointOnLine.pose.position.x = midpoints[0];
    closestPointOnLine.pose.position.y = midpoints[1];
}

// sort array to start at closest point on path
void sortPathArray(void)
{
    // Initialize placeHolder
    geometry_msgs::PoseArray placeHolder;
    geometry_msgs::Pose tempArray[pathPose.poses.size()];
    for(int i = 0; i < pathPose.poses.size(); i++)
    {
        placeHolder.poses.push_back(tempArray[i]);
    }
    for(int i = 0; i < pathPose.poses.size(); i++)
    {
      (placeHolder.poses[i]).position.x = 0;
      (placeHolder.poses[i]).position.y = 0;
    }
    
    for (int i = 0; i < pathPose.poses.size(); i++)
    {
        placeHolder.poses[i].position.x = pathPose.poses[i].position.x;
        placeHolder.poses[i].position.y = pathPose.poses[i].position.y;
    }
    
    pathPose.poses[0].position.x = pathPose.poses[closestPointIndex].position.x;
    pathPose.poses[0].position.y = pathPose.poses[closestPointIndex].position.y;
    int j = 1;
    int z = 0;
    while ( !(pathPose.poses[closestPointIndex + j].position.x == 0 && pathPose.poses[closestPointIndex + j].position.y == 0) )
    {
        pathPose.poses[j].position.x = pathPose.poses[closestPointIndex+j].position.x;
        pathPose.poses[j].position.y = pathPose.poses[closestPointIndex+j].position.y;
        j++;
    }
    for (z = 0 ; z < closestPointIndex; z++)
    {
        pathPose.poses[j].position.x = placeHolder.poses[z].position.x;
        pathPose.poses[j].position.y = placeHolder.poses[z].position.y;
        j++;
    }
}

void findIndexOfLastPointOnPath(void)
{
    int i;
    for(i = 0; i < pathPose.poses.size(); i++)
    {
        if ( (pathPose.poses[i].position.x == 0) && (pathPose.poses[i].position.y == 0) )
        {
            break;
        }    
    }
    if (i != 0) 
    {
    lastPointOnPathIndex = i - 1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower_test_copy"); //Ros Initialize
    ros::start();
    ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)

    ros::NodeHandle n;
    ros::Subscriber pathSub;
    ros::Subscriber poseEstSub;
    ros::Publisher goalPub;
    ros::Publisher velPub;

    pathSub = n.subscribe<geometry_msgs::PoseArray>("/path", 1, pathCallback);
    poseEstSub = n.subscribe<geometry_msgs::PoseStamped>("/poseEstimation", 1, poseEstCallback);
    goalPub = n.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1000, true);
    velPub = n.advertise<geometry_msgs::Twist>("/path_vel", 1000, true);
   
    // Initialize msgs and flags
    newPath = false;
    closestPointOnLine.pose.position.x = 0;
    closestPointOnLine.pose.position.y = 0;
    constVelTerm.linear.x = 0;
    constVelTerm.linear.y = 0;
    constVelTerm.linear.z = 0;
    constVelTerm.angular.x = 0;
    constVelTerm.angular.y = 0;
    constVelTerm.angular.z = 0;
    
    // count variables
    int open_path_count = 0;
    int count = 0;
    
    while (ros::ok()) 
    {
        ros::spinOnce();
        //ROS_INFO("spinned\n\n");
        
        if(newPath) // check if a new path has been set
        {
            //ROS_INFO("in new path\n\n");
            findIndexOfLastPointOnPath();
            //ROS_INFO("found last index\n\n");
            if(isOpenLoop) // path given is OPEN
            {
            	std::cout << "\n\n" << "Path count: " << open_path_count++ << "\n\n";
                newPath = false; // reset flag
                goalPose.pose = (pathPose.poses)[0]; // publish first point on path
                goalPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                goalPub.publish(goalPose);
                while( distanceFormula(pathPose.poses[0].position.x, poseEst.pose.position.x, 
                                        pathPose.poses[0].position.y, poseEst.pose.position.y) >= BOUNDARY_RADIUS ) // FIXME: Implement this sleep cycle as a function
                    {
                        ros::spinOnce();
                        loop_rate.sleep();
                        if(newPath || !ros::ok())
                        {
                            break;
                        }
                    }
                closestPointIndex = 0; // initialize to first point in path 
                count = 0;
                while(closestPointIndex != lastPointOnPathIndex) // use interpolation
                {
                    ros::spinOnce();
                    if(newPath || !ros::ok()) // FIXME: break out if a different pose is published
                    {
                        break;
                    }
                    //ROS_INFO("on interpolation loop OPEN\n");
                    //checkDistanceTraveled();
                    calcClosestPointOnPath();
                    findClosestPointOnLine();
                    closestPointOnLine.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                    calcConstVelTerm();
                    std::cout << "---------------------------------------------------------------------\n";
                    std::cout << "Loop Count: " << count++ << "\n";
                    std::cout << "Closest Point Index: " << closestPointIndex << "\n";
                    std::cout << "Last Point Index: " << lastPointOnPathIndex << "\n";
                    std::cout << "Line Distance: " << line_dist << "\n";
                    std::cout << "Line Distance Traveled: " << line_dist_trav << "\n\n";
                    std::cout << "Pose Est:\n" << poseEst << "\n\n";
                    std::cout << "Goal Pose:\n" << closestPointOnLine << "\n\n";
                    std::cout << "Constant Vel:\n" << constVelTerm << "\n\n";
                    std::cout << "---------------------------------------------------------------------\n\n";
                    velPub.publish(constVelTerm);
                    goalPub.publish(closestPointOnLine);
                    pathPose.poses[closestPointIndex].position.x = 0;
                    pathPose.poses[closestPointIndex].position.y = 0;
                    loop_rate.sleep();
                }
                goalPose.pose = (pathPose.poses)[lastPointOnPathIndex]; // publish final point on path
                goalPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                goalPub.publish(goalPose);
                pathPose.poses[closestPointIndex].position.x = 0;
                pathPose.poses[closestPointIndex].position.y = 0;
		constVelTerm.linear.x = 0;
		constVelTerm.linear.y = 0;
		velPub.publish(constVelTerm);
            }
            else // path given is CLOSED
            {
                newPath = false; // reset flag and set stopping condition for while loop
                calcClosestPointOnPath();
                /*ROS_INFO("closest point calculated\n\n");
                std::cout << "---------------------------------------------------------------------\n";
                std::cout << "Pose Est:\n" << poseEst << "\n\n";
                std::cout << "Closest Point Index: " << closestPointIndex << "\n\n";
                std::cout << "Last Point Index: " << lastPointOnPathIndex << "\n\n";
                std::cout << "Path Array:\n" << pathPose << "\n\n";*/
                sortPathArray(); // array now starts at the closest point index
                //std::cout << "Sorted Path Array:\n" << pathPose << "\n\n";
                //std::cout << "---------------------------------------------------------------------\n";
                closestPointIndex = 0;
                while( !newPath || ros::ok() ) // while no new path has been published
                {
                    ROS_INFO("on interpolation loop CLOSED");
                    findClosestPointOnLine();
                    closestPointOnLine.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                    calcConstVelTerm();
                    std::cout << "Goal pose:\n" << closestPointOnLine << "\n\n";
                    std::cout << "Constant vel:\n" << constVelTerm << "\n\n";
                    velPub.publish(constVelTerm);
                    goalPub.publish(closestPointOnLine);
                    calcClosestPointOnPath();
                    if (closestPointIndex == lastPointOnPathIndex)
                    {
                        closestPointIndex = 0;
                    }
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
        }
        //ROS_INFO("END\n---------------------------------------------\n\n");
        loop_rate.sleep();
    }
}
//END
