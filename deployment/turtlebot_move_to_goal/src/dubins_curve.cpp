// dubins_curve.cpp
// Shengdong Liu
// Spring 2015
//
// subscribe to the current position of the TurtleBot
// subscribe to the goal position 
// publish the Dubins path to "amcl_path" 

#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
#include "dubins.h"
#include "nav_msgs/Path.h"
#include "PoseWithName.h"

#define PI 3.14159265358978
//#define TOPIXELS 231  //approximate pixels per meter at 2.7 meter high

//TODO find the minimum turning radius
const double mtr = 0.02222222222222;
//TODO find the best step size of the path sample
const double stepSize = 0.0008888888888;
//counter to determine when to publish the complete path
int counter = 0;


//fill the nav_msgs:Path with positions
int makePath(double q[3], double x, void* user_data) {
	printf("(%f,%f,%f)\n", q[0], q[1], q[2]);
	nav_msgs::Path * dubinsPath = (nav_msgs::Path *)(user_data);
	counter = dubinsPath->poses.size();	
	dubinsPath->poses.resize(counter + 1);
	dubinsPath->poses[counter].pose.position.x = q[0];
	dubinsPath->poses[counter].pose.position.y = q[1];
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(q[2]);
	dubinsPath->poses[counter].pose.orientation = odom_quat;
	//std::cout << counter <<std::endl;
	return 0;
}

class DubinsCurve
{
	public:
		DubinsCurve();
	private:

		// Methods
		void poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr&);
		void goalCallback(const geometry_msgs::PoseStampedConstPtr&);
                double xToMeter(double);
                double yToMeter(double);
                
		// ROS 
		ros::NodeHandle ph_, nh_;
		ros::Subscriber pos_sub_;
		ros::Subscriber goal_sub_;
		ros::Publisher path_pub_;

		// Other member variables
		geometry_msgs::PoseStamped goalPose_;
		geometry_msgs::TwistPtr cmd_vel_;
		nav_msgs::Path dubinsPath;

		bool got_goal_;
		int path_size_;
		double angle_tolerance_;
		double distance_tolerance_;
		double prev_goal_[3];
};

DubinsCurve::DubinsCurve():
	cmd_vel_(new geometry_msgs::Twist),
	angle_tolerance_(3.14),             // aka. angle doesn't matter
	distance_tolerance_(0.075),

	got_goal_(false),
	path_size_(1)
{

	// ROS publisher and subscribers
	pos_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("afterKalman", 1, &DubinsCurve::poseCallback, this);
	goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",1, &DubinsCurve::goalCallback, this);
	path_pub_ = nh_.advertise<nav_msgs::Path>("amcl_path", 1, true); 
}

/**
 * This Callback function gets the goal position and store it in the goalPose_
 * variable and set got_goal_ variable to true
 */
void DubinsCurve::goalCallback(const geometry_msgs::PoseStampedConstPtr& goalPose){
	goalPose_ = *goalPose;
	got_goal_ = true;
}

/**
 * This Callback function gets the current position use it in conjunction with
 * goalPose_ to create a dubins path. The path is then published.
 */
void DubinsCurve::poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& pose)
{
	if ( got_goal_ ){

		double cur_pose[3] = { xToMeter(pose->pose.position.x),
			               yToMeter(pose->pose.position.y),
			               tf::getYaw(pose->pose.orientation)};
		double goal_pose[3] = {	goalPose_.pose.position.x,
			                goalPose_.pose.position.y,
			                tf::getYaw(goalPose_.pose.orientation)};

		//std::cout << "heading = " << orientation << "\n";
		//std::cout << "my_x = " << pose->pose.pose.position.x << " ;my_y = " << pose->pose.pose.position.y << "\n";
		//std::cout << "goal_x = " << goalPose_.pose.position.x << " ;goal_y = " << goalPose_.pose.position.y << "\n";

		// Recalculate if the goal changed
		if ( fabs(goal_pose[0] - prev_goal_[0]) >= distance_tolerance_ 
                  || fabs(goal_pose[1] - prev_goal_[1]) >= distance_tolerance_ ){
		    //abs(goal_pose[2] - prev_goal_[2]) >= angle_tolerance_  ) {


                    //automatically assign goal orientation
                    goal_pose[2] = atan( (goal_pose[1] - cur_pose[1])/
                                         (goal_pose[0] - cur_pose[0]) );

                    if ( goal_pose[0] - cur_pose[0] < 0 ){
                      goal_pose[2] += PI;
                    }
                      
		    

			counter = 0;

			// create a dubins path (see dubins.cpp)
			DubinsPath path;
			int err = dubins_init(cur_pose, goal_pose, mtr, &(path));

			double pathLen = dubins_path_length(&path);
			path_size_ = int(pathLen/stepSize) + 1;
			dubinsPath.poses.resize(0);

			// sample the path to get a vector of positions
			dubins_path_sample_many( &(path), makePath, stepSize, &dubinsPath );	

                        //publish the path
			path_pub_.publish(dubinsPath); 
			// save the prev_goal_ to check if goal change in next iteration
			prev_goal_[0] = goal_pose[0];
			prev_goal_[1] = goal_pose[1];
			prev_goal_[2] = goal_pose[2];
		}
	}
	else {
		//ROS_WARN("No goal received yet");
		got_goal_ = false;
	}
}

double DubinsCurve::xToMeter(double x)
{
	return (x - 301.0)/900.0;
}

double DubinsCurve::yToMeter(double y)
{
	return (y - 247.0)/900.0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "DubinsCurve");
	DubinsCurve dubinsCurve;

	ros::spin();
}
