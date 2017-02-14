#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
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

const int T = 1;

int main(int argc, char **argv)
{
	tf2_msgs::TFMessage centroidPositions;
	geometry_msgs::PoseArray cDot;
	geometry_msgs::Pose pose;
	geometry_msgs::TransformStamped transform_stamped;
	transform_stamped.transform.translation.x = 0;
    	transform_stamped.transform.translation.y = 0;
    
	ros::init(argc, argv, "CoM_sample_input_test"); 
	ros::start();
	ros::Rate loop_rate(T);
	ros::NodeHandle nh_;
	ros::Publisher centroid_pub;
	ros::Publisher cDot_pub;

	centroid_pub = nh_.advertise<tf2_msgs::TFMessage>("/poseEstimationC", 1000, true);
	cDot_pub = nh_.advertise<geometry_msgs::PoseArray>("/gauss", 1000, true);

	cDot.poses.push_back(pose);
	cDot.poses[0].position.z = 0;

	for(int i = 0; i < 50; i++)
	{
        	centroidPositions.transforms.push_back(transform_stamped);
    	}

  	while (ros::ok())
  	{
		//ros::spinOnce();

		centroid_pub.publish(centroidPositions);
		cDot_pub.publish(cDot);
		loop_rate.sleep();
	}
}


